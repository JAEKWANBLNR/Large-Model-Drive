#!/usr/bin/env bash
# Configure non-secret AWS settings for Large Model Drive.

set -euo pipefail
umask 077

config_dir="${XDG_CONFIG_HOME:-${HOME}/.config}/large-model-drive"
env_file="${config_dir}/env"
mkdir -p "${config_dir}"

echo "Configure credentials with 'aws configure', an IAM role, or another"
echo "standard boto3 credential provider. Use least-privilege S3, Transcribe,"
echo "and Polly permissions; this script never stores AWS secret keys."

read -r -p "AWS region [ap-southeast-1]: " aws_region
aws_region="${aws_region:-ap-southeast-1}"
read -r -p "S3 bucket used for temporary transcription audio: " bucket_name
if [[ -z "${bucket_name}" ]]; then
  echo "A bucket name is required." >&2
  exit 1
fi

temporary_file="$(mktemp "${config_dir}/env.XXXXXX")"
trap 'rm -f "${temporary_file}"' EXIT
if [[ -f "${env_file}" ]]; then
  grep -v -E '^export (AWS_REGION|AWS_S3_BUCKET)=' "${env_file}" \
    > "${temporary_file}" || true
fi
printf 'export AWS_REGION=%q\n' "${aws_region}" >> "${temporary_file}"
printf 'export AWS_S3_BUCKET=%q\n' "${bucket_name}" >> "${temporary_file}"
mv "${temporary_file}" "${env_file}"
chmod 600 "${env_file}"
trap - EXIT

echo "Non-secret AWS settings saved to ${env_file}."
echo "Load them before launching: source \"${env_file}\""
