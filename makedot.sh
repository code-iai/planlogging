#!/bin/bash

for f in packaging/*.dot; do
    filename=$(basename "$f")
    extension="${filename##*.}"
    filename="${filename%.*}"
    dot -Tpdf $f > packaging/${filename}.pdf
done
