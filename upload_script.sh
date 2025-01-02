#!/bin/bash

set -x

# Navigate to the repository
cd /home/arjuna/Fire_Fighting_Robot

# Add all changes (files, modifications, deletions)
git add .

# Commit the changes
git commit -m "Automated upload: $(date)"

# Push the changes to the main repository
git push origin main

# Update submodules and push their changes
git submodule update --init --recursive
git submodule foreach 'git push origin main'

