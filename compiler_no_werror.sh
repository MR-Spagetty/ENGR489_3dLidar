#!/bin/bash
set -e

# Packages can explicitly add -Werror after CMake's global compiler flags.
args=()
for arg in "$@"; do
  case "$arg" in
    -Werror|-Werror=*) ;;
    *) args+=("$arg") ;;
  esac
done

exec "${args[@]}" -Wno-error
