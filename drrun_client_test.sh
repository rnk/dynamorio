#!/bin/bash
set -e
rm -rf logs/*
make > /dev/null
./bin64/drrun -debug -ops '-loglevel 3 -msgbox_mask 0x1' \
  -client suite/tests/bin/lib${1}.dll.so 0 "" \
  suite/tests/bin/${1}
