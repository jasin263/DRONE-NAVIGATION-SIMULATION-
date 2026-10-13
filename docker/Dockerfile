FROM python:3.12-slim

WORKDIR /app

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# Install basic OS utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy packaging manifests and install dependencies
COPY pyproject.toml README.md ./
RUN pip install --upgrade pip && \
    pip install ".[dev]"

# Copy source code and configuration files
COPY src/ ./src/
COPY configs/ ./configs/
COPY docs/ ./docs/

# Install the project
RUN pip install -e .

EXPOSE 8000

ENTRYPOINT ["drone-nav"]
CMD ["viewer", "--config", "configs/presets/medium.yaml", "--host", "0.0.0.0", "--port", "8000"]
