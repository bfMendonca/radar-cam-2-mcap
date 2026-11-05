# syntax=docker/dockerfile:1
# Build stage: resolve the locked dependencies (the ld2451 driver is a git dependency, so git
# is needed) and install the package, non-editable, into /app/.venv.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim AS build
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy UV_PYTHON_DOWNLOADS=0
RUN apt-get update \
    && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY pyproject.toml uv.lock README.md LICENSE ./
COPY src ./src
RUN uv sync --locked --no-default-groups --no-editable

# Runtime stage: same Python minor version, only the virtualenv, a non-root user.
FROM python:3.15-slim-bookworm
RUN useradd --create-home --uid 1000 converter
COPY --from=build /app/.venv /app/.venv
ENV PATH="/app/.venv/bin:$PATH"
USER converter
WORKDIR /home/converter
ENTRYPOINT ["radar-cam-2-mcap"]
CMD ["--help"]
