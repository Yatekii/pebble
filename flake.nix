{
  description = "Pebble ESP32-C6 firmware development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { nixpkgs, rust-overlay, ... }:
    let
      forAllSystems = function:
        nixpkgs.lib.genAttrs [
          "aarch64-darwin"
          "x86_64-darwin"
          "x86_64-linux"
          "aarch64-linux"
        ] (system: function system);
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ rust-overlay.overlays.default ];
          };

          rustToolchain = pkgs.rust-bin.nightly.latest.default.override {
            extensions = [ "rust-src" "rust-analyzer" ];
            targets = [ "riscv32imac-unknown-none-elf" ];
          };

          # Xcode 26.6 (17F113)'s metal/metallib driver rejects the only Metal
          # Toolchain Apple ships (17F109) over a version-match bug, breaking
          # gpui's shader build. The actual compiler works and is mounted in a
          # cryptex; shadow xcrun to route metal/metallib there, pass the rest
          # through. Drop once Apple ships a matching toolchain.
          metalShim = pkgs.writeShellScriptBin "xcrun" ''
            tool=""
            for a in "$@"; do case "$a" in metal|metallib) tool="$a"; break;; esac; done
            if [ -n "$tool" ]; then
              bin="$(ls -d /private/var/run/com.apple.security.cryptexd/mnt/com.apple.MobileAsset.MetalToolchain-*/Metal.xctoolchain/usr/bin 2>/dev/null | sort | tail -1)"
              if [ -n "$bin" ] && [ -x "$bin/$tool" ]; then
                args=(); seen=0
                for a in "$@"; do
                  if [ "$seen" = 1 ]; then args+=("$a"); continue; fi
                  [ "$a" = "$tool" ] && seen=1
                done
                exec "$bin/$tool" "''${args[@]}"
              fi
            fi
            exec /usr/bin/xcrun "$@"
          '';
        in
        {
          default = pkgs.mkShell {
            # metalShim must precede system paths so it shadows /usr/bin/xcrun.
            nativeBuildInputs = pkgs.lib.optional pkgs.stdenv.isDarwin metalShim;

            buildInputs = with pkgs; [
              rustToolchain
              probe-rs-tools
              pkg-config
            ];

            shellHook = ''
              echo "Pebble ESP32-C6 development environment"
              echo "Rust toolchain: nightly with riscv32imac-unknown-none-elf target"
            '';
          };
        }
      );
    };
}
