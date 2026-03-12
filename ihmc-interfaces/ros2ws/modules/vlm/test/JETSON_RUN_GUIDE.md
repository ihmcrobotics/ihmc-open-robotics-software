# Running the Explosive Breaching Demo on Jetson Orin (JetPack 6.2)

Target hardware: **NVIDIA Jetson AGX Orin 64 GB**
JetPack: **6.2** (L4T R36.4) · CUDA: **12.6** · GPU arch: **sm_87** (Tegra Ampere)

> For the x86 dev machine setup see [RUN_GUIDE.md](RUN_GUIDE.md).

---

## Prerequisites

- JetPack 6.2 fully installed (`jtop` or `nvcc --version` should report CUDA 12.6)
- Docker with NVIDIA container runtime:
  ```bash
  # Verify
  docker run --rm --runtime=nvidia ubuntu nvidia-smi
  ```
  If the command fails, install the runtime:
  ```bash
  sudo apt install nvidia-container-toolkit
  sudo systemctl restart docker
  ```
- ROS 2 Humble installed on the Jetson
- Workspace built with `compile_interfaces.bash` (see below)

---

## Step 0: Build the ROS 2 Workspace (one-time)

Do **not** have a virtual environment active when building.

```bash
cd ~/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
source /opt/ros/humble/setup.bash
./compile_interfaces.bash
```

Ensure `modules/ros2-env/COLCON_IGNORE` exists so colcon skips the venv:
```bash
touch modules/ros2-env/COLCON_IGNORE
```

---

## Backend Options

Two backends are available. Choose one per run.

### Option A — vLLM (higher throughput, requires Docker)

#### Step 1: Pull the Jetson vLLM Docker image (one-time, ~10 GB)

```bash
docker pull dustynv/vllm:r36.4-cu129-24.04
```

This image is compiled for `sm_87` (Tegra Ampere). Standard `pip install vllm` is
**not supported** on Jetson — do not attempt it.

#### Step 2: Download the model (one-time)

The Docker container reads the model from `$HOME/.cache/huggingface`, which is
mounted into the container automatically by the startup script.

```bash
# Install huggingface_hub in your venv or system Python
pip install huggingface_hub

huggingface-cli download Qwen/Qwen2.5-VL-7B-Instruct-AWQ \
    --local-dir $HOME/.cache/huggingface/hub/Qwen2.5-VL-7B-Instruct-AWQ
```

Expected size: ~4 GB (AWQ quantized).

#### Step 3: Start the vLLM server (Terminal 1)

```bash
cd ~/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
bash modules/vlm/test/start_qwen_vl_server_jetson.sh
```

Wait until you see:
```
Application startup complete.
```

The server listens on `http://localhost:8000` (OpenAI-compatible API) — identical
to the dev machine setup, so `behavior_coordinator.py` requires no changes.

If the container runs out of memory, lower GPU utilization:
```bash
bash modules/vlm/test/start_qwen_vl_server_jetson.sh --gpu-memory-utilization 0.70
```

#### Step 4: Run the coordinator (Terminal 2)

```bash
cd ~/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
source modules/ros2-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=32
python3 modules/vlm/behavior-interface/behavior_coordinator.py
```

`behavior_coordinator.py` uses `VLMInterface` (vLLM) by default — no import change needed.

---

### Option B — Ollama (easier setup, no Docker required)

Ollama supports aarch64 natively and `qwen2.5vl` is confirmed working on JetPack 6.2.

#### Step 1: Install Ollama (one-time)

```bash
curl -fsSL https://ollama.com/install.sh | sh
# Ollama starts as a systemd service automatically — no manual server start needed
```

#### Step 2: Pull the model (one-time)

```bash
ollama pull qwen2.5vl   # pulls the 7B variant on a 64 GB device
```

#### Step 3: Switch the coordinator to Ollama

In `behavior_coordinator.py` change line 32:
```python
# Before (vLLM):
from vlm_interface import VLMInterface

# After (Ollama):
from ollama_interface import OllamaInterface as VLMInterface
```

#### Step 4: Run the coordinator

```bash
cd ~/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
source modules/ros2-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=32
python3 modules/vlm/behavior-interface/behavior_coordinator.py
```

---

## Key Differences vs. Dev Machine

| | Dev Machine (RTX 4070) | Jetson Orin 64 GB |
|---|---|---|
| GPU arch | sm_89 (Ada Lovelace) | sm_87 (Tegra Ampere) |
| CUDA | 12.8 | 12.6 |
| vLLM install | `pip install vllm` | Docker only (`dustynv/vllm:r36.4-cu129-24.04`) |
| vLLM server script | `start_qwen_vl_server.sh` | `start_qwen_vl_server_jetson.sh` |
| Model cache location | `modules/vlm/test/models/` | `$HOME/.cache/huggingface/` |
| `behavior_coordinator.py` | Unchanged | Unchanged (vLLM) or swap import (Ollama) |
| `--max-num-seqs` | 64 | 16 (less concurrent memory headroom) |

---

## Stopping

- **vLLM**: Press `Ctrl+C` in Terminal 1 (Docker container exits cleanly)
- **Ollama**: `sudo systemctl stop ollama` (or leave running — it idles at ~0% GPU)
- **Coordinator**: `Ctrl+C` in Terminal 2

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `docker: unknown flag --runtime=nvidia` | Install `nvidia-container-toolkit` and restart Docker |
| Container starts but hangs at model load | GPU memory too low — retry with `--gpu-memory-utilization 0.70` |
| `CUDA error: no kernel image for this device` | Wrong Docker image — must use `dustynv/vllm:r36.4-*`, not the x86 vLLM image |
| `pip install vllm` fails on Jetson | Expected — use Docker instead |
| `No module named 'behavior_msgs'` | Rebuild workspace and re-source `install/setup.bash` |
| `No module named 'openai'` or `ollama'` | Activate `ros2-env` and run `pip install openai ollama` |
| Ollama: `qwen2.5vl` not using GPU | Known issue with some tags — try `ollama pull qwen2.5vl:7b` explicitly |
| colcon build errors from venv | Ensure `modules/ros2-env/COLCON_IGNORE` exists |
| Commands not reaching robot | Check `ROS_DOMAIN_ID` matches on both machines |
