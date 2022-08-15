#!/bin/bash

main () {
set -e
cd unittests
make clean && make && make unit_tests
cd -

set +e
checkLeaks ./unittests/bin/mocklock_tests
checkLeaks ./unittests/bin/lockless_tests
}

checkLeaks () {
binary=$1
valgrind --leak-check=full $binary 2>&1 >/dev/null  | grep "no leaks are possible"
if [ $? != 0 ]
then
	echo -e "\e[1;31mTHERE ARE LEAKS !! In $binary\e[0m"
	exit 1
fi
}

main
