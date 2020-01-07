#!/usr/bin/bash
mkdir -p data/json

# handle data/config files
cue export data/config.cue > data/json/config.json
