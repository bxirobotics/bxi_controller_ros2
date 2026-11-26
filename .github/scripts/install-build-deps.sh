#!/usr/bin/env bash
set -euo pipefail

sudo sed -i \
  -e 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' \
  -e 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' \
  -e 's|http://ports.ubuntu.com/ubuntu-ports|https://ports.ubuntu.com/ubuntu-ports|g' \
  /etc/apt/sources.list

# A mirror can change between fetching InRelease and Packages. Retry both together.
for attempt in 1 2 3; do
  if sudo apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update; then
    break
  fi
  if (( attempt == 3 )); then
    exit 1
  fi
  delay=$((15 * attempt))
  echo "::warning::APT index update failed; retrying in ${delay}s."
  sleep "$delay"
done

sudo apt-get -o Acquire::Retries=3 install -y "$@"
