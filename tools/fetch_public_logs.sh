#!/usr/bin/env bash
# Fetches the public real flight logs listed in logs/public/manifest.tsv for
# gate 3. Each file is pinned to a commit and checked against its sha256, so
# every run replays exactly the same bytes. The logs belong to the teams that
# flew them; most carry no licence, so they are downloaded for testing and
# never committed here (see logs/public/README.md).
#
#   tools/fetch_public_logs.sh [OUT_DIR]     default: build/public-logs
#
# A log that can't be fetched (network, repository gone) is reported and
# skipped; grreplay's --min-flights then decides whether enough are left.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$ROOT/build/public-logs}"
MANIFEST="$ROOT/logs/public/manifest.tsv"
mkdir -p "$OUT"

urlencode() { python3 -c 'import sys, urllib.parse; print(urllib.parse.quote(sys.argv[1]))' "$1"; }

ok=0 cached=0 failed=0
while IFS=$'\t' read -r name repo commit path sha licence; do
  [[ -z "${name:-}" || "$name" == \#* ]] && continue
  dest="$OUT/$name"
  if [ -f "$dest" ] && echo "$sha  $dest" | sha256sum -c --status; then
    cached=$((cached + 1))
    continue
  fi
  url="https://raw.githubusercontent.com/$repo/$commit/$(urlencode "$path")"
  if curl -sSfL --retry 3 --retry-delay 2 -o "$dest.part" "$url" && echo "$sha  $dest.part" | sha256sum -c --status; then
    mv "$dest.part" "$dest"
    ok=$((ok + 1))
  else
    echo "warning: could not fetch $name from $repo@${commit:0:7}" >&2
    rm -f "$dest.part"
    failed=$((failed + 1))
  fi
done < "$MANIFEST"
echo "public logs: $ok downloaded, $cached already present, $failed failed -> $OUT"
