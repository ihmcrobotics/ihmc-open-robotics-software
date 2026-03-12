#!/bin/bash
# Start Qwen2.5-7B-Instruct-AWQ via vLLM (OpenAI-compatible API server)
# Serves on http://localhost:8000

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../ros2-env/bin/activate"

MODEL_PATH="$SCRIPT_DIR/models/Qwen2.5-7B-Instruct-AWQ"

vllm serve "$MODEL_PATH" \
  --host 0.0.0.0 \
  --port 8000 \
  --max-model-len 4096 \
  --gpu-memory-utilization 0.85 \
  --dtype float16 \
  --max-num-seqs 64
