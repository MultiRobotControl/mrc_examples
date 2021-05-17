#!/bin/bash

usage()
{
  echo "Usage:  set_executable.sh  ROOT_DIRECTORY"
  echo ""
  echo "Sets the executable bit on a number of file types"
}

if [ $# -lt 1 ]; then
  echo "ERROR - you need to supply the root directory."
  usage
  exit 1
fi

EXTS=("*.py" "*.sh")
for EXT in "${EXTS[@]}"
do 
  find $1 -type f -name ${EXT} -print -exec chmod +x "{}" \;
done
  
