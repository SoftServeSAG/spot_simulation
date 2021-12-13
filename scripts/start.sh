#!/bin/bash

IGNITION_ROOT=~/.ignition/fuel

mkdir -p "$IGNITION_ROOT"

if [ ! -f "$IGNITION_ROOT/config.yaml" ]; then
    cp config.yaml "$IGNITION_ROOT/"
fi

if_is_link=`readlink $0`
command_name=$0
if [[ if_is_link ]]; then
  command_name=$if_is_link
fi

MY_PATH=`dirname "$if_is_link"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

# start tmuxinator
tmuxinator
