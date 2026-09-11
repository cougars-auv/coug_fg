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
while true; do
  selected_bags=$(cd "${BAGS_DIR}" && find . -name "metadata.yaml" -exec dirname {} \; |
    sed 's|^\./||' | sort -r |
    gum choose --no-limit --header "Select bags to evaluate:") || exit 0
  [[ -n ${selected_bags} ]] && break
done

mapfile -t selected_bags <<<"${selected_bags}"
bag_paths=()
for bag in "${selected_bags[@]}"; do
  bag_paths+=("${BAGS_DIR}/${bag}")
done

while true; do
  selected_agents=$(basename -a "${CONFIG_DIR}"/*_params.yaml |
    sed 's/_params.yaml$//' | sort |
    gum choose --no-limit --header "Select agents to evaluate:") || exit 0
  [[ -n ${selected_agents} ]] && break
done

mapfile -t selected_agents <<<"${selected_agents}"

# --- Options ---
evo_options=$(gum choose --no-limit --header "Select evo flags:" -- \
  "--align" \
  "--project_to_plane xy") || exit 0
evo_flags=$(printf '%s\n' "${evo_options}" | tr '\n' ' ')

# --- Run ---
run_args=(
  --bags "${bag_paths[@]}"
  --agents "${selected_agents[@]}"
  "--evo-flags=${evo_flags}"
)

echo "python3 $(dirname "$0")/eval_bags.py ${run_args[*]}"
python3 "$(dirname "$0")/eval_bags.py" "${run_args[@]}"
