#!/bin/bash
ip="$(cat pi.ip)"
ssh -t pi@"$ip" "cd botlab-f18; bash"
