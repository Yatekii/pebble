//! HTTP client for tile loading.

use futures::future::BoxFuture;
use http_client::{AsyncBody, HttpClient, Request, Response};

use std::sync::Arc;

/// HTTP client using reqwest (rustls, no OpenSSL/curl).
///
/// reqwest/hyper need a Tokio reactor to drive I/O, but gpui polls our returned
/// future on its own executor. So we own a Tokio runtime and run each request on
/// it via `spawn`, awaiting the join handle from whatever executor polls us.
pub struct ReqwestHttpClient {
    client: reqwest::Client,
    runtime: Arc<tokio::runtime::Runtime>,
}

impl ReqwestHttpClient {
    pub fn new() -> anyhow::Result<Arc<Self>> {
        let runtime = Arc::new(
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()?,
        );
        let client = reqwest::Client::builder()
            .user_agent("PebbleGUI/1.0")
            .build()?;

        Ok(Arc::new(Self { client, runtime }))
    }
}

impl HttpClient for ReqwestHttpClient {
    fn send(
        &self,
        req: Request<AsyncBody>,
    ) -> BoxFuture<'static, anyhow::Result<Response<AsyncBody>>> {
        let client = self.client.clone();
        let runtime = self.runtime.clone();

        Box::pin(async move {
            let (parts, _body) = req.into_parts();

            let handle = runtime.spawn(async move {
                let method = reqwest::Method::from_bytes(parts.method.as_str().as_bytes())?;
                let mut builder = client.request(method, parts.uri.to_string());
                for (key, value) in parts.headers.iter() {
                    builder = builder.header(key.as_str(), value.as_bytes());
                }

                let response = builder.send().await?;
                let status = response.status();
                let headers = response.headers().clone();
                let bytes = response.bytes().await?;
                Ok::<_, anyhow::Error>((status, headers, bytes))
            });

            let (status, headers, bytes) = handle.await??;

            let mut builder = http_client::http::Response::builder().status(status.as_u16());
            for (key, value) in headers.iter() {
                builder = builder.header(key.as_str(), value.as_bytes());
            }

            let response = builder
                .body(AsyncBody::from_bytes(bytes))
                .map_err(|e| anyhow::anyhow!("{}", e))?;

            Ok(response)
        })
    }

    fn user_agent(&self) -> Option<&http_client::http::HeaderValue> {
        None
    }

    fn proxy(&self) -> Option<&http_client::Url> {
        None
    }
}
