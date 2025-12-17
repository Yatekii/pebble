//! HTTP client for tile loading.

use futures::future::BoxFuture;
use futures::io::AsyncReadExt;
use gpui_http_client::{AsyncBody, HttpClient, Request, Response};

use std::sync::Arc;

/// Simple HTTP client using isahc.
pub struct IsahcHttpClient {
    client: isahc::HttpClient,
}

impl IsahcHttpClient {
    pub fn new() -> Arc<Self> {
        let client = isahc::HttpClient::builder()
            .default_headers(&[("User-Agent", "PebbleGUI/1.0")])
            .build()
            .expect("Failed to create HTTP client");

        Arc::new(Self { client })
    }
}

impl HttpClient for IsahcHttpClient {
    fn send(
        &self,
        req: Request<AsyncBody>,
    ) -> BoxFuture<'static, anyhow::Result<Response<AsyncBody>>> {
        let client = self.client.clone();

        Box::pin(async move {
            let (parts, _body) = req.into_parts();

            let mut builder = isahc::Request::builder()
                .method(parts.method.as_str())
                .uri(parts.uri.to_string());

            for (key, value) in parts.headers.iter() {
                builder = builder.header(key.as_str(), value.to_str().unwrap_or(""));
            }

            let isahc_req = builder.body(()).map_err(|e| anyhow::anyhow!("{}", e))?;

            let response = client
                .send_async(isahc_req)
                .await
                .map_err(|e| anyhow::anyhow!("{}", e))?;

            let status = response.status();
            let headers = response.headers().clone();

            let mut body = response.into_body();
            let mut bytes = Vec::new();
            body.read_to_end(&mut bytes)
                .await
                .map_err(|e| anyhow::anyhow!("{}", e))?;

            let mut builder = gpui_http_client::http::Response::builder().status(status.as_u16());
            for (key, value) in headers.iter() {
                builder = builder.header(key.as_str(), value.to_str().unwrap_or(""));
            }

            let response = builder
                .body(AsyncBody::from_bytes(bytes.into()))
                .map_err(|e| anyhow::anyhow!("{}", e))?;

            Ok(response)
        })
    }

    fn user_agent(&self) -> Option<&gpui_http_client::http::HeaderValue> {
        None
    }

    fn proxy(&self) -> Option<&gpui_http_client::Url> {
        None
    }

    fn type_name(&self) -> &'static str {
        "IsahcHttpClient"
    }
}
