#!/bin/bash

echo starting monitor...

GREY="\033[0;37m"
YELLOW="\033[0;34m"

tput civis

cat /dev/ttyUSB0 | while read -n1 c; do
  echo -e -n "$GREY[$(date +%T)] $YELLOW"
  echo -n "$c" | hexdump -e '1/1 "0x%2x "' -e '1/1 "%c\n"'
done
