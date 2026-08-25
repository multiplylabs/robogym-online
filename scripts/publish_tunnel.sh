#!/usr/bin/env bash
# Expose a local generator publicly, and point the published page at it.
#
# The page looks for `stream.json` beside itself and drives whatever generator it names, so the
# published link never has to change. A quick tunnel's hostname does change every time it starts,
# which is exactly why the address lives in a hundred-byte file rather than in the bundle: moving
# the generator costs a commit, not a rebuild and redeploy of an eighty-megabyte site.
#
# With no generator running the same link is still the recorded-clip demo. Nothing breaks when this
# machine is off; the page simply stops being steerable.
#
#     python -m robogym_online.wasd_server --generator onnx    # in another terminal
#     scripts/publish_tunnel.sh
set -euo pipefail

port="${1:-8765}"
log="$(mktemp)"

cloudflared tunnel --url "http://localhost:${port}" --no-autoupdate >"${log}" 2>&1 &
tunnel=$!
trap 'kill "${tunnel}" 2>/dev/null || true' EXIT

echo "waiting for a hostname..."
host=""
for _ in $(seq 1 30); do
  host="$(grep -ao 'https://[a-z0-9-]*\.trycloudflare\.com' "${log}" | head -1 || true)"
  if [ -n "${host}" ]; then break; fi
  sleep 2
done

if [ -z "${host}" ]; then
  echo "cloudflared produced no hostname; see ${log}" >&2
  exit 1
fi

printf '{"url": "wss://%s"}\n' "${host#https://}" >stream.json
echo "stream.json now reads: $(cat stream.json)"
echo
echo "Commit and push it, and the published link steers this machine:"
echo "    git add stream.json && git commit -m 'Point the page at a generator' && git push"
echo
echo "Tunnel is up. Ctrl-C ends it, and the link reverts to the recorded clip."
wait "${tunnel}"
