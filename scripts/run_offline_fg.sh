#!/bin/bash
set -e

# --- Selection ---
while true; do
  bags=$(cd "${BAGS_DIR}" && find . -name "metadata.yaml" -exec dirname {} \; | \
    sed 's|^\./||' | sort -r | \
    gum choose --no-limit --header "Select bags to process offline...") || exit 0
  [ -n "${bags}" ] && break
done

namespace=$(basename -a "${CONFIG_DIR}"/*_params.yaml | \
  sed 's/_params.yaml$//' | sort | \
  gum filter --placeholder "Select an agent namespace...") || exit 0
[ -z "${namespace}" ] && exit 0

# --- Options ---
tag=$(gum input --placeholder "Set output tag..." || true)
tag="${tag:-offline}$(date +'_%Y-%m-%d-%H-%M-%S')"

evo_options=$(gum choose --no-limit --header "Select evo flags:" -- \
  "--align" \
  "--project_to_plane xy") || exit 0
evo_flags="${evo_options//$'\n'/ }"

bag_paths=()
for b in ${bags}; do
  bag_paths+=("${BAGS_DIR}/${b}")
done

# --- Process ---
python3 "$(dirname "$0")/run_offline_fg.py" \
  --bags "${bag_paths[@]}" \
  --namespace "${namespace}" \
  --tag "${tag}" \
  --evo-flags="${evo_flags}"
