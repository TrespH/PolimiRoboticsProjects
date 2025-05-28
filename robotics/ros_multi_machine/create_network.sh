#!/bin/bash
docker network create --subnet 172.28.0.0/16 ros-net
docker network create --subnet 172.29.0.0/16 other-net

