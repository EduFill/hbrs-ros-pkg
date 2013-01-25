#!/bin/bash
# Param1: pc to shutdown
# Param2: -r or -h
# need to execute "sudo chmod +s /sbin/shutdown" once on the machines which you want shutdown remotely
 
ssh $1 "shutdown" $2 "now" &
