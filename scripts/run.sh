#!/usr/bin/env bash
set -euo pipefail       # -e: exit on error, -u: treat unset vars as errors, -o pipefail: catch pipe errors

echo "=== Running simp_perm.sh ==="
./extras.sh

echo "=== Running simp_perm.sh ==="
./permissions.sh

echo "=== Running build.sh ==="
./build.sh

