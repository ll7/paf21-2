#!/bin/bash

if [[ -z ${paf_dir+z} ]]; then
  SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
  export paf_dir="$SCRIPT_DIR/../../"
fi
update-alternatives --list python 1>/dev/null
if [ $? -ne 0 ]; then
  echo "sudo required for installing python executable in update-alternatives"
  sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 100
fi
if ! (update-alternatives --get-selections | grep /usr/bin/python3.8); then
  echo "sudo required for setting python executable in update-alternatives"
  sudo update-alternatives --set python /usr/bin/python3.8
fi
