#!/usr/bin/env bash

cd "$(dirname "$0")" || exit
if [ "$(basename "$(pwd)")" != "equations" ]; then
    exit 1
fi

fd -e tex -x pdflatex -shell-escape {}
fd -e aux -e fdb_latexmk -e fls -e log -e pdf -X rm
