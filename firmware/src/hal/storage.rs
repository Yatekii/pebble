//! Non-volatile flash storage for puzzle box state.
//!
//! This module provides persistent storage using a dedicated flash sector
//! that is not overwritten during normal firmware updates. Data is serialized
//! using postcard format with CRC32 for integrity checking.
//!
//! # Flash Layout
//!
//! The ESP32-C6-MINI-1 module has 4MB of flash. We reserve the last sector
//! (4KB) for state storage at offset 0x3FF000 (4MB - 4KB).
//!
//! This location is safe because:
//! - The ESP-IDF bootloader and partition table occupy the first ~64KB
//! - The application partition is typically 1-2MB
//! - The last sector at 4MB-4KB is well beyond any normal firmware
//!
//! # Storage Format
//!
//! ```text
//! +--------+--------+--------+--------+------------------+
//! | Magic  | Magic  | Length | Length |      Data        |
//! | 0x50   | 0x42   |  LSB   |  MSB   | (postcard + CRC) |
//! +--------+--------+--------+--------+------------------+
//!    "P"      "B"      u16 LE          serialized data
//! ```
//!
//! The data section contains postcard-serialized data followed by a CRC32.
//!
//! # Usage
//!
//! ```ignore
//! use esp_storage::FlashStorage;
//!
//! let flash = FlashStorage::new();
//! let mut storage = Storage::new(flash);
//! let mut buf = [0u8; SECTOR_SIZE];
//!
//! // Load state (returns default if not found or corrupted)
//! let state: PuzzleState = storage.load(&mut buf).unwrap_or_default();
//!
//! // Save state
//! storage.save(&state, &mut buf)?;
//! ```

use defmt::Format;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use postcard::{from_bytes, to_slice};
use serde::{Deserialize, Serialize};

/// Flash sector size (4KB).
pub const SECTOR_SIZE: usize = 4096;

/// Offset of the storage sector from the start of flash.
///
/// Located at the last sector of a 4MB flash (4MB - 4KB = 0x3FF000).
/// This is well beyond the typical application size and won't be
/// overwritten during normal firmware updates.
pub const STORAGE_OFFSET: u32 = 0x3F_F000;

/// Header size: 2 bytes magic + 2 bytes length.
const HEADER_SIZE: usize = 4;

/// CRC32 size in bytes.
const CRC_SIZE: usize = 4;

/// Maximum size of serialized data (sector size minus header and CRC).
pub const MAX_DATA_SIZE: usize = SECTOR_SIZE - HEADER_SIZE - CRC_SIZE;

/// Magic bytes to identify valid stored data ("PB" for PuzzleBox).
const MAGIC: [u8; 2] = [0x50, 0x42];

/// Storage errors.
#[derive(Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Flash read error.
    Read,
    /// Flash write error.
    Write,
    /// Flash erase error.
    Erase,
    /// Data too large for storage sector.
    DataTooLarge,
    /// No valid data found (magic mismatch or empty).
    NotFound,
    /// CRC mismatch (data corrupted).
    CrcMismatch,
    /// Deserialization failed (corrupted data).
    Corrupted,
    /// Serialization failed.
    SerializationFailed,
}

/// Non-volatile storage backed by flash.
///
/// Generic over the flash driver to allow testing with mock implementations.
pub struct Storage<F> {
    flash: F,
}

impl<F> Storage<F>
where
    F: ReadNorFlash + NorFlash,
{
    /// Create a new storage instance.
    pub fn new(flash: F) -> Self {
        Self { flash }
    }

    /// Load data from flash.
    ///
    /// Returns `Err(Error::NotFound)` if no valid data exists.
    /// Returns `Err(Error::CrcMismatch)` if data exists but CRC check fails.
    /// Returns `Err(Error::Corrupted)` if deserialization fails.
    pub fn load<'de, T>(&mut self, buf: &'de mut [u8; SECTOR_SIZE]) -> Result<T, Error>
    where
        T: Deserialize<'de>,
    {
        // Read the entire sector
        self.flash
            .read(STORAGE_OFFSET, buf)
            .map_err(|_| Error::Read)?;

        // Check magic bytes
        if buf[0..2] != MAGIC {
            return Err(Error::NotFound);
        }

        // Get data length (includes CRC)
        let total_len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
        if !(CRC_SIZE..=MAX_DATA_SIZE + CRC_SIZE).contains(&total_len) {
            return Err(Error::Corrupted);
        }

        let data_len = total_len - CRC_SIZE;
        let data_start = HEADER_SIZE;
        let data_end = data_start + data_len;
        let crc_start = data_end;

        // Verify CRC
        let stored_crc = u32::from_le_bytes([
            buf[crc_start],
            buf[crc_start + 1],
            buf[crc_start + 2],
            buf[crc_start + 3],
        ]);
        let computed_crc = crc32(&buf[data_start..data_end]);

        if stored_crc != computed_crc {
            return Err(Error::CrcMismatch);
        }

        // Deserialize data
        let data = &buf[data_start..data_end];
        from_bytes(data).map_err(|_| Error::Corrupted)
    }

    /// Save data to flash.
    ///
    /// This erases the storage sector and writes the new data with CRC.
    pub fn save<T>(&mut self, data: &T, buf: &mut [u8; SECTOR_SIZE]) -> Result<(), Error>
    where
        T: Serialize,
    {
        // Clear buffer (0xFF is erased flash state)
        buf.fill(0xFF);

        // Serialize data after header, leaving room for CRC
        let data_start = HEADER_SIZE;
        let data_buf = &mut buf[data_start..SECTOR_SIZE - CRC_SIZE];
        let serialized = to_slice(data, data_buf).map_err(|_| Error::SerializationFailed)?;
        let data_len = serialized.len();

        if data_len > MAX_DATA_SIZE {
            return Err(Error::DataTooLarge);
        }

        // Compute and append CRC
        let crc = crc32(&buf[data_start..data_start + data_len]);
        let crc_start = data_start + data_len;
        buf[crc_start..crc_start + 4].copy_from_slice(&crc.to_le_bytes());

        // Write header (total length includes CRC)
        let total_len = data_len + CRC_SIZE;
        buf[0] = MAGIC[0];
        buf[1] = MAGIC[1];
        buf[2] = (total_len & 0xFF) as u8;
        buf[3] = ((total_len >> 8) & 0xFF) as u8;

        // Erase sector
        self.flash
            .erase(STORAGE_OFFSET, STORAGE_OFFSET + SECTOR_SIZE as u32)
            .map_err(|_| Error::Erase)?;

        // Write data (only write up to the next 4-byte aligned boundary after our data)
        let write_len = align_up(HEADER_SIZE + total_len, 4);
        self.flash
            .write(STORAGE_OFFSET, &buf[..write_len])
            .map_err(|_| Error::Write)?;

        Ok(())
    }

    /// Erase all stored data.
    pub fn erase(&mut self) -> Result<(), Error> {
        self.flash
            .erase(STORAGE_OFFSET, STORAGE_OFFSET + SECTOR_SIZE as u32)
            .map_err(|_| Error::Erase)
    }

    /// Check if valid data exists without fully loading it.
    ///
    /// This only checks the magic bytes, not the CRC.
    pub fn has_data(&mut self) -> Result<bool, Error> {
        let mut header = [0u8; 4];
        self.flash
            .read(STORAGE_OFFSET, &mut header)
            .map_err(|_| Error::Read)?;

        Ok(header[0..2] == MAGIC)
    }

    /// Get a reference to the underlying flash driver.
    pub fn flash(&self) -> &F {
        &self.flash
    }

    /// Get a mutable reference to the underlying flash driver.
    pub fn flash_mut(&mut self) -> &mut F {
        &mut self.flash
    }
}

/// Align a value up to the next multiple of `align`.
const fn align_up(value: usize, align: usize) -> usize {
    (value + align - 1) & !(align - 1)
}

/// Compute CRC32 (IEEE 802.3 polynomial) of data.
///
/// This is a simple implementation without lookup tables to save flash space.
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;

    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320; // IEEE 802.3 polynomial (reversed)
            } else {
                crc >>= 1;
            }
        }
    }

    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32() {
        // Test vector: "123456789" should produce 0xCBF43926
        let data = b"123456789";
        assert_eq!(crc32(data), 0xCBF4_3926);
    }

    #[test]
    fn test_align_up() {
        assert_eq!(align_up(0, 4), 0);
        assert_eq!(align_up(1, 4), 4);
        assert_eq!(align_up(4, 4), 4);
        assert_eq!(align_up(5, 4), 8);
    }
}
