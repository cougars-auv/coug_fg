# 🌊 CoUGARs Multi-Agent Factor Graph Odometry

[![ROS 2 Build & Test](https://github.com/cougars-auv/coug_fg/actions/workflows/ros2_build_and_test.yml/badge.svg)](https://github.com/cougars-auv/coug_fg/actions/workflows/ros2_build_and_test.yml)
[![Docker Build](https://github.com/cougars-auv/coug_fg/actions/workflows/docker_build.yml/badge.svg)](https://github.com/cougars-auv/coug_fg/actions/workflows/docker_build.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/coug_fg/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/coug_fg/main)
[![codecov](https://codecov.io/gh/cougars-auv/coug_fg/graph/badge.svg?token=OFBQI4IOT1)](https://codecov.io/gh/cougars-auv/coug_fg)

## Contributing

We **strongly recommend** using the [`cougars-dev`](https://github.com/cougars-auv/cougars-dev/tree/main#contributing) development environment.

## Releasing

This repository follows the **Semantic Versioning (SemVer 2.0.0)** standard:
> Given a version number **`MAJOR.MINOR.PATCH`**, increment the:
> - **MAJOR** version when you make incompatible API changes
> - **MINOR** version when you add functionality in a backward compatible manner
> - **PATCH** version when you make backward compatible bug fixes

- **Tag and Push:** Create and push a version tag (e.g., `v1.2.3`):

  ```bash
  git tag v1.2.3
  git push origin v1.2.3
  ```

  Pushing the tag automatically creates a draft GitHub Release with auto-generated notes.

- **Publish a GitHub Release:** Review the draft release in GitHub and click **Publish**.

## Citations

If you use this repository in your research, please cite the following publications:

### CoUGARs
```bibtex
@misc{durrant2025lowcostmultiagentfleetacoustic,
  title={Low-cost Multi-agent Fleet for Acoustic Cooperative Localization Research},
  author={Nelson Durrant and Braden Meyers and Matthew McMurray and Clayton Smith and Brighton Anderson and Tristan Hodgins and Kalliyan Velasco and Joshua G. Mangelson},
  year={2025},
  eprint={2511.08822},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.08822},
}
```
