#!/bin/bash
echo "Finishing all softwares ..."
docker rm -f $(docker ps -aq)
echo "Done!"
