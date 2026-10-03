// Cross-origin isolation for a static host.
//
// WebAssembly threads need SharedArrayBuffer, and a browser grants that only to a page served
// with two headers, Cross-Origin-Opener-Policy and Cross-Origin-Embedder-Policy. GitHub Pages
// cannot send them. A service worker can add them to every response it hands the page, so this
// file runs twice: once as a page script, which registers the worker and reloads if the page is
// not yet isolated, and once as the worker, which stamps the headers onto what it fetches.
//
// Where the server already sends the headers (the local preview does) the page is isolated on
// first load and this registers nothing.

const HEADERS = {
  "Cross-Origin-Opener-Policy": "same-origin",
  "Cross-Origin-Embedder-Policy": "require-corp",
};

if (typeof window === "undefined") {
  // Service-worker context.
  self.addEventListener("install", () => self.skipWaiting());
  self.addEventListener("activate", (event) => event.waitUntil(self.clients.claim()));
  self.addEventListener("fetch", (event) => {
    const request = event.request;
    // Only same-origin GETs: the page's own files. Nothing else needs the headers, and a cached
    // "only-if-cached" request cannot be re-issued.
    if (request.method !== "GET" || request.cache === "only-if-cached" || new URL(request.url).origin !== self.location.origin) {
      return;
    }
    event.respondWith(
      fetch(request).then((response) => {
        if (response.status === 0) return response;
        const headers = new Headers(response.headers);
        for (const [name, value] of Object.entries(HEADERS)) headers.set(name, value);
        return new Response(response.body, { status: response.status, statusText: response.statusText, headers });
      })
    );
  });
} else if (!window.crossOriginIsolated && "serviceWorker" in navigator && window.isSecureContext) {
  // Page context, not isolated: install the worker and reload once it controls this page, so the
  // document itself is re-fetched with the headers. The flag stops a loop if the headers never
  // arrive (an unsupported browser, or a host that strips them).
  const script = document.currentScript?.src;
  const FLAG = "isolation-worker-reloaded";
  if (script && !sessionStorage.getItem(FLAG)) {
    const reload = () => {
      sessionStorage.setItem(FLAG, "1");
      window.location.reload();
    };
    navigator.serviceWorker.addEventListener("controllerchange", reload);
    navigator.serviceWorker
      .register(script)
      .then((registration) => {
        // Installed on an earlier visit but not controlling this load: no controllerchange will
        // come, so reload now.
        if (registration.active && !navigator.serviceWorker.controller) reload();
      })
      .catch((error) => console.warn("[isolation-worker] not installed:", error));
  }
}
