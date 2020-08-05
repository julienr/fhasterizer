#!/bin/bash
for directory in src
do
  echo "Running clang-format under ${directory}"
  find "${directory}" \( -name '*.h' -or -name '*.cpp' \) -print0 | xargs -0 clang-format --style=file -i
done
