#!/bin/bash
# Start Qwen2.5-VL-7B-Instruct-AWQ via vLLM on NVIDIA Jetson Orin (JetPack 6.2 / L4T R36.4)
# Uses the dusty-nv jetson-containers Docker image compiled for sm_87 (Tegra Ampere).
# Supports both text and image inputs. Serves on http://localhost:8000.
#
# Prerequisites:
#   - Docker with NVIDIA container runtime installed (nvidia-container-toolkit)
#   - Model downloaded to $HOME/.cache/huggingface  (see JETSON_RUN_GUIDE.md)
#
# Usage:
#   bash start_qwen_vl_server_jetson.sh
#   bash start_qwen_vl_server_jetson.sh --gpu-memory-utilization 0.70   # if OOM

set -e

DOCKER_IMAGE="dustynv/vllm:r36.4-cu129-24.04"
MODEL="Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
PORT=8000
GPU_MEM_UTIL=${1:-0.80}   # override with first argument, e.g. 0.70

# Parse --gpu-memory-utilization flag if passed
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gpu-memory-utilization)
            GPU_MEM_UTIL="$2"; shift 2 ;;
        *)
            shift ;;
    esac
done

echo "Starting vLLM server on Jetson Orin..."
echo "  Image : $DOCKER_IMAGE"
echo "  Model : $MODEL"
echo "  Port  : $PORT"
echo "  GPU utilization: $GPU_MEM_UTIL"
echo ""
echo "Wait for: 'Application startup complete.'"
echo ""

docker run --rm -it \
  --network host \
  --runtime=nvidia \
  --shm-size=16g \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -v "$HOME/.cache/huggingface:/root/.cache/huggingface" \
  "$DOCKER_IMAGE" \
  vllm serve "$MODEL" \
    --host 0.0.0.0 \
    --port "$PORT" \
    --max-model-len 4096 \
    --gpu-memory-utilization "$GPU_MEM_UTIL" \
    --dtype float16 \
    --max-num-seqs 16 \
    --limit-mm-per-prompt '{"image": 4}'
