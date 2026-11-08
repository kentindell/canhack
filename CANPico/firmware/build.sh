#!/usr/bin/env bash
# Moved to the repository root: use ./build.sh there.
# This wrapper forwards to it so old invocations keep working.
here="$(cd "$(dirname "$0")" && pwd)"
exec "$here/../../build.sh" "$@"
