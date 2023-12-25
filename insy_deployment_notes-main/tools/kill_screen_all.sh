#!/bin/bash
# This script closes all open screen terminals
# It uses the command screen -ls to list the screen terminals
# It uses the command screen -S id -X quit to close each screen terminal
# It uses a for loop to iterate over the ids of the screen terminals

# Get the ids of the screen terminals
ids=$(screen -ls | grep -oP '\d+(?=\.)')

# Iterate over the ids and close each screen terminal
for id in $ids; do
  screen -S $id -X quit
done

# Show a confirmation message
echo "All screen terminals have been closed."

