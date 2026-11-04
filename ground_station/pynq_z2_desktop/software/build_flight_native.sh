#!/bin/bash
set -eu
flight_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
gcc -O3 -std=c11 -fPIC -shared -o "$flight_root/_flight_instruments.new.so" "$flight_root/flight_native.c"
mv "$flight_root/_flight_instruments.new.so" "$flight_root/_flight_instruments.so"
echo FLIGHT_NATIVE_BUILT
