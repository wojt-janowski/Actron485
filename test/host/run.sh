#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
test_bin=$(mktemp "${TMPDIR:-/tmp}/actron-protocol.XXXXXX")
trap 'rm -f "$test_bin"' EXIT
c++ -std=c++17 -DHEX=16 -Wno-switch -Wno-vla-cxx-extension -g -fsanitize=address,undefined -Itest/host/stubs -Iinclude test/host/protocol_test.cpp src/*.cpp -o "$test_bin"
"$test_bin"
