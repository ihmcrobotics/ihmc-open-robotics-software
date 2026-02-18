# VLM — Vision-Language Model Integration

Uses **Qwen2.5-VL** (served locally via vLLM) to drive robot behavior in the IHMC explosive breaching demo.
The model exposes an OpenAI-compatible API, so the Python client uses the standard `openai` SDK.

## Directory Structure

```
vlm/
├── behavior-interface/       # ROS 2 behavior coordinator and VLM client
│   ├── vlm_interface.py          # Reusable VLM client wrapper
│   ├── behavior_coordinator.py   # ROS 2 node — mission planning and command dispatch
│   ├── behavior_coordinator.ipynb# Interactive notebook version of the coordinator
│   ├── config.json               # Mission planner prompt
│   ├── config_goto.json          # GOTO parameter extractor prompt
│   ├── config_scan.json          # SCAN target extractor prompt (vision-enabled)
│   └── config_receive_object.json# RECEIVE OBJECT parameter extractor prompt
└── test/                     # Server startup scripts, notebooks, and test images
    ├── start_qwen_vl_server.sh   # Launch vLLM server (text + image)
    ├── start_qwen_server.sh      # Launch vLLM server (text only)
    ├── RUN_GUIDE.md              # Step-by-step run instructions
    ├── qwen2.5_7b_instruct_demo.ipynb
    ├── qwen3_vl_8b_demo.ipynb
    ├── test_qwen_client.py
    └── room_expo.png             # Test image for vision calls
```

## How It Works

### VLMInterface (`vlm_interface.py`)

A thin wrapper around the OpenAI SDK. Each instance is configured by a JSON file that defines:
- The system prompt, few-shot examples, and output format schema
- Model settings (`temperature`, `max_tokens`, etc.)
- Whether to attach an image (`supports_vision`, `image`)

On each `call_model(vlm_input)` call, the runtime context (`vlm_input`) is prepended to the static
prompt assembled from config, and the full message is sent to the local vLLM server.
All calls are logged to `<model_name>/<prompt_type>/vlm_logs.json`.

### BehaviorCoordinator (`behavior_coordinator.py`)

A ROS 2 node that subscribes to robot status and publishes behavior commands.

```
/ihmc/behavior_tree/ai2r_status  →  BehaviorCoordinator  →  /ihmc/behavior_tree/ai2r_command
```

**Control loop** (runs on each status message):

1. Wait until the robot is idle (`behavior_in_progress == "-"`)
2. Wait until the last issued command is acknowledged as complete
3. On first idle tick: call the **mission planner VLM** to build an ordered behavior queue
4. Pop the next behavior and call a **per-behavior VLM** to extract typed parameters
5. Publish the command and repeat

### The Four VLM Roles

| Role | Config | Output |
|---|---|---|
| Mission planner | `config.json` | `behavior_list = [SCAN, GOTO(...), RECEIVE OBJECT(...), ...]` |
| GOTO params | `config_goto.json` | JSON: target object, approach direction, spatial discriminators |
| SCAN targets | `config_scan.json` | JSON: list of objects the robot should look for (optionally vision-grounded) |
| RECEIVE OBJECT params | `config_receive_object.json` | JSON: object name and hand side (0=left, 1=right) |

### Spatial Object Resolution

When a GOTO target is ambiguous (e.g. multiple `Person` instances in the scene), the VLM returns
spatial cues (`spatially_related_object`, `spatial_relation_obj`, `class_discriminator`) that are
resolved geometrically using the robot's real-world pose and 3D dot products against its heading.

## Config File Schema

All four configs share the same structure:

```jsonc
{
    "api_base": "http://localhost:8000/v1",
    "model": "auto",            // "auto" queries the server for the loaded model name
    "temperature": 0.0,
    "max_tokens": 512,
    "supports_vision": false,   // true enables base64 image attachment
    "image": null,              // default image path for vision calls
    "prompt_type": "...",       // used as the log subdirectory name
    "system_prompt": [...],
    "task_description": [...],
    "examples": [...],
    "immediate_task": "...",
    "precognition": "...",      // chain-of-thought hint ("Think step by step...")
    "output_format": [...]
}
```

Array fields are joined with newlines; plain strings are used as-is.

## Demo Mission

The task configured in `config.json` is the explosive breaching demo sequence:

1. **SCAN** — scan the environment to discover scene objects
2. **GOTO** — navigate to the person to the right of the barrier
3. **RECEIVE OBJECT** — receive the explosive charge from the person
4. **GOTO** — navigate to the front of the DoorPanel
5. **PLACE CHARGE ON DOOR** — attach the charge (no extra params needed)
6. **GOTO** — navigate behind the barrier

## Setup

### 1. Create the virtual environment

From the `ros2ws` root (do **not** have ROS sourced yet):

```bash
python3 -m venv modules/ros2-env
touch modules/ros2-env/COLCON_IGNORE   # prevent colcon from scanning the venv
```

### 2. Install Python dependencies

```bash
source modules/ros2-env/bin/activate
pip install vllm openai numpy scipy ipykernel
# Register the venv as a Jupyter kernel (needed for the notebook)
python3 -m ipykernel install --user --name ros2-env --display-name "ros2-env"
```

### 3. Download the model

The startup scripts expect the model at `modules/vlm/test/models/<model-name>/`.
Download Qwen2.5-VL-7B-Instruct-AWQ from Hugging Face:

```bash
mkdir -p modules/vlm/test/models
source modules/ros2-env/bin/activate
pip install huggingface_hub
huggingface-cli download Qwen/Qwen2.5-VL-7B-Instruct-AWQ \
    --local-dir modules/vlm/test/models/Qwen2.5-VL-7B-Instruct-AWQ
```

### 4. Build the ROS 2 workspace (one-time, or after interface changes)

Do **not** have the virtual environment active when building:

```bash
source /opt/ros/humble/setup.bash
./compile_interfaces.bash
```

### 5. Start the vLLM server

Two scripts are provided depending on whether you need image support:

| Script | Model | Vision |
|---|---|---|
| `test/start_qwen_vl_server.sh` | Qwen2.5-VL-7B-Instruct-AWQ | Yes (up to 4 images/prompt) |
| `test/start_qwen_server.sh` | Qwen2.5-7B-Instruct-AWQ | No |

```bash
# From ros2ws root — activates ros2-env internally
bash modules/vlm/test/start_qwen_vl_server.sh
```

Wait until you see:
```
Application startup complete.
```

The server listens on `http://localhost:8000` (OpenAI-compatible API).

**Key server flags** (edit the script to change):

| Flag | Default | Description |
|---|---|---|
| `--max-model-len` | 4096 | Max context length in tokens |
| `--gpu-memory-utilization` | 0.85 | Fraction of GPU VRAM to use; lower if OOM |
| `--dtype` | float16 | Compute dtype |
| `--max-num-seqs` | 64 | Max concurrent sequences |
| `--limit-mm-per-prompt` | `{"image": 4}` | Max images per prompt (VL server only) |

## Running

See [`test/RUN_GUIDE.md`](test/RUN_GUIDE.md) for full run instructions.

**Quick start** (after setup is complete):

```bash
# Terminal 1 — start the vLLM server
bash modules/vlm/test/start_qwen_vl_server.sh
# Wait for: "Application startup complete."

# Terminal 2 — source environment and run coordinator
source modules/ros2-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=32
python3 modules/vlm/behavior-interface/behavior_coordinator.py
```
