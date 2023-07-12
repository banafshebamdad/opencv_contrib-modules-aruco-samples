#!/bin/bash

# Mi Jul 12, 2023 12:08:47 CET
# Banafshe Bamdad
# usage: source banafshe_update_repo.sh [commit_message]
# This script push all changes into github repository

if [ "$#" -eq 0 ]
then
  echo "Usage: source banafshe_update_repo.sh [commit_message]"
  exit 1
fi

commint_msg=$1
git add .
git commit -m "$commint_msg"
git push -u origin master
