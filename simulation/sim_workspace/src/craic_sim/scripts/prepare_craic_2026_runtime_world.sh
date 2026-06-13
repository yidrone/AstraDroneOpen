#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATOR="${SCRIPT_DIR}/generate_craic_2026_world.py"
OUTPUT=""
RING_X=""
SEED=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      OUTPUT="$2"
      shift 2
      ;;
    --ring-x)
      RING_X="$2"
      shift 2
      ;;
    --seed)
      SEED="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "$OUTPUT" ]]; then
  echo "--output is required" >&2
  exit 1
fi

GEN_ARGS=(--output "$OUTPUT")
if [[ -n "$RING_X" ]]; then
  GEN_ARGS+=(--ring-x "$RING_X")
fi
if [[ -n "$SEED" ]]; then
  GEN_ARGS+=(--seed "$SEED")
fi

python3 "$GENERATOR" "${GEN_ARGS[@]}" >&2
printf '%s\n' "$OUTPUT"
