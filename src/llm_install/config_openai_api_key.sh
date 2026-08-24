#!/usr/bin/env bash
# Store OPENAI_API_KEY in a user-only environment file.

set -euo pipefail
umask 077

config_dir="${XDG_CONFIG_HOME:-${HOME}/.config}/large-model-drive"
env_file="${config_dir}/env"
mkdir -p "${config_dir}"

read -r -s -p "Enter your OpenAI API key: " api_key
echo
if [[ -z "${api_key}" ]]; then
  echo "No key entered; nothing changed." >&2
  exit 1
fi

temporary_file="$(mktemp "${config_dir}/env.XXXXXX")"
trap 'rm -f "${temporary_file}"' EXIT
if [[ -f "${env_file}" ]]; then
  grep -v '^export OPENAI_API_KEY=' "${env_file}" > "${temporary_file}" || true
fi
printf 'export OPENAI_API_KEY=%q\n' "${api_key}" >> "${temporary_file}"
mv "${temporary_file}" "${env_file}"
chmod 600 "${env_file}"
trap - EXIT

echo "Key saved to ${env_file} with mode 600."
echo "Load it before launching: source \"${env_file}\""
