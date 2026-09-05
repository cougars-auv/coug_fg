#!/bin/bash
# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

# --- Selection ---
mapfile -t leaf_dirs < <(
  cd "${BAGS_DIR}"
  find . -name "metadata.yaml" -exec dirname {} \; | sed 's|^\./||'
)

dir_list=(bags)
for d in "${leaf_dirs[@]}"; do
  p="${d}"
  while [[ ${p} != . && -n ${p} ]]; do
    dir_list+=("${p}")
    p=$(dirname "${p}")
  done
done

selected_dir=$(printf '%s\n' "${dir_list[@]}" | sort -u | \
  gum filter --placeholder "Select directory or bag to evaluate ('bags' for all)...") || exit 0
[[ -z ${selected_dir} ]] && exit 0

if [[ ${selected_dir} == bags ]]; then
  target_dir="${BAGS_DIR}"
else
  target_dir="${BAGS_DIR}/${selected_dir}"
fi

while true; do
  selected_agents=$(basename -a "${CONFIG_DIR}"/*_params.yaml | \
    sed 's/_params.yaml$//' | sort | \
    gum choose --no-limit --header "Select agents to evaluate...") || exit 0
  [[ -n ${selected_agents} ]] && break
done

# --- Options ---
evo_options=$(gum choose --no-limit --header "Select evo flags:" -- \
  "--align" \
  "--project_to_plane xy") || exit 0
evo_flags=$(printf '%s\n' "${evo_options}" | tr '\n' ' ')

mapfile -t selected_agents <<< "${selected_agents}"

# --- Evaluate ---
python3 "$(dirname "$0")/eval_bags.py" \
  --target-dir "${target_dir}" \
  --agents "${selected_agents[@]}" \
  --evo-flags="${evo_flags}"
