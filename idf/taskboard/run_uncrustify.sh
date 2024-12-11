#! /bin/bash

# Get script path
script_path=$(cd $(dirname $0); pwd)

# Run uncrustify
find $script_path/main \( -name "*.cpp" -o -name "*.hpp" \) -exec uncrustify -c $script_path/uncrustify.cfg --no-backup {} +