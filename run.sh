#!/bin/bash

# rodar comando totalmente desnecessario para ativar o sudo
sudo echo "sudo ativado";

readonly OUTRO_SCRIPT="run2.sh"; 

chmod +x $OUTRO_SCRIPT;
# nohup[$OUTRO_SCRIPT] > /dev/null &
gnome-terminal -x bash -c "./$OUTRO_SCRIPT; exec $SHELL";

cd 'Main software'
# cmake -S . -B build
cd build
# make
./K1D