FROM python:3.12-slim

WORKDIR /app

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# Install basic system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy package definition and application files
COPY pyproject.toml README.md ./
COPY src/ ./src/
COPY configs/ ./configs/
COPY docs/ ./docs/

# Upgrade pip and install the drone-nav package in editable mode
RUN pip install --upgrade pip && \
    pip install -e .

EXPOSE 8000

ENTRYPOINT ["drone-nav"]
CMD ["viewer", "--config", "medium", "--host", "0.0.0.0", "--port", "8000"]
