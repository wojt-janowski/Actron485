#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
test_bin=$(mktemp "${TMPDIR:-/tmp}/actron-protocol.XXXXXX")
trap 'rm -f "$test_bin"' EXIT
for test_source in test/host/*_test.cpp; do
  c++ -std=c++17 -DHEX=16 -Wno-switch -Wno-vla-cxx-extension -g -fsanitize=address,undefined -Itest/host/stubs -Iinclude "$test_source" src/*.cpp -o "$test_bin"
  "$test_bin"
done
