#!/bin/bash
set -euo pipefail

readonly scriptDir=$(dirname $0)

function main
{
  export BIN=$scriptDir/unittests/bin # the test suite (build_system.py) expects binaries to be here
  export CFLAGS="--coverage"
  export CXXFLAGS="--coverage"
  export LDFLAGS="--coverage"

  rm -rf "$BIN"
  mkdir "$BIN"

  run_test_suite

  lcov -q \
    --no-external \
    --capture \
    --directory unittests \
	--directory common \
	--directory al5e \
	--directory al5d \
	--directory include \
    --directory $BIN \
    --output-file $BIN/profile.info

  lcov -r "$BIN/profile.info" "test/*" -o "$BIN/profile.info" > /dev/null

  genhtml $BIN/profile.info --prefix=$PWD --output-directory $BIN/cov-html

  firefox "$BIN/cov-html/index.html"
}

function run_test_suite
{
	./check.sh
}

main "$@"

