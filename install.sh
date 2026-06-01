#!/bin/bash
# Deploy vendored packages from git-tracked local files to system Python 3.10 site-packages.
set -e

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
SITEDIR="/home/zjj/.local/lib/python3.10/site-packages"

echo "Deploying openarm_driver → $SITEDIR/openarm_driver/"
cp -v "$REPO_ROOT/src/local_openarm_driver/"*.py   "$SITEDIR/openarm_driver/"
cp -v "$REPO_ROOT/src/local_openarm_driver/"*.yaml "$SITEDIR/openarm_driver/"

echo "Deploying dora_openarm → $SITEDIR/dora_openarm/"
cp -v "$REPO_ROOT/src/local_dora_openarm/main.py"  "$SITEDIR/dora_openarm/"

echo "Done. Run: dora run config/dataflow-real.yaml"
