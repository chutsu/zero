#!/bin/bash

# Run clang-format on source
if [ -d "src" ]; then
  cd src || exit
  find . -name "*.c" -print0 \
    | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
  cd ..
fi

# Run clang-format on header
if [ -d "include" ]; then
  cd include || exit
  find . -name "*.h" -print0 \
    | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
  cd ..
fi

# Run clang-format on tests
if [ -d "tests" ]; then
  cd tests || exit
  find . -name "*.c" -print0 \
    | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
  cd ..
fi
