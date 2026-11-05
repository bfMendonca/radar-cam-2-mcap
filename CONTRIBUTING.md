# Contributing

Bug reports, field results from real recordings and pull requests are welcome.

## Setup

```sh
git clone https://github.com/bfMendonca/radar-cam-2-mcap.git
cd radar-cam-2-mcap
uv sync --all-groups
```

## Before opening a pull request

These are the checks CI runs (`.github/workflows/ci.yml`):

```sh
uv run ruff check .
uv run ruff format --check .
uv run mypy --strict src
uv run pytest -q
```

The full test suite takes a few minutes. `uv run mypy` also checks `tests`.

- Keep the topic tables in the README and `docs/output-format.md`, `config/default.toml` and `mapping.py` in step.
- Changes to the output contract should bump `CONTRACT_VERSION` in `src/radar_cam_2_mcap/__init__.py`.
- Add an entry under *Unreleased* in `CHANGELOG.md`.
- The source session folder is read-only: the converter must never modify it.
