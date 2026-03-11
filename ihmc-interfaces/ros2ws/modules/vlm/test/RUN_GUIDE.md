# Running the Explosive Breaching Demo with Qwen VL

## Prerequisites

- ROS 2 Humble installed
- `ros2-env` virtual environment exists at `ros2ws/modules/ros2-env/`
- vllm and openai installed in `ros2-env`
- Workspace built with `compile_interfaces.bash`
- Qwen2.5-VL model downloaded at `ros2ws/modules/vlm/test/models/Qwen2.5-VL-7B-Instruct-AWQ/`

## Building the Workspace (one-time, or after interface changes)

Do NOT activate the virtual environment when building.

```bash
cd ~/alex/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
source /opt/ros/humble/setup.bash
./compile_interfaces.bash
```

Make sure `modules/ros2-env/COLCON_IGNORE` exists so colcon skips the venv directory.

## Step 1: Start the vLLM Server (Terminal 1)

```bash
cd ~/alex/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
bash modules/vlm/test/start_qwen_vl_server.sh
```

The script activates `ros2-env` internally. Wait until you see:

```
Application startup complete.
```

## Step 2: Launch VS Code (Terminal 2)

Source everything, then open VS Code so it inherits the environment:

```bash
cd ~/alex/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws
source modules/ros2-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=32
code .
```

## Step 3: Run the Notebook in VS Code

1. Open `modules/qwen_vl_client.ipynb`
2. Select the **ros2-env** kernel (top right kernel picker)
   - If not listed: Select Another Kernel > Python Environments > enter path:
     `modules/ros2-env/bin/python3`
3. Run all cells from the top

## Stopping

- **Notebook**: Interrupt the kernel (stop button) to stop the demo
- **vLLM server**: Press `Ctrl+C` in Terminal 1

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `No module named 'behavior_msgs'` | Rebuild workspace and re-source `install/setup.bash` |
| `No module named 'openai'` | Activate `ros2-env` and run `pip install openai` |
| scipy/numpy errors | Run `pip install --force-reinstall scipy>=1.11` inside `ros2-env` |
| GPU memory error | Lower `--gpu-memory-utilization` in `start_qwen_vl_server.sh` or close GPU-heavy apps |
| Commands not reaching robot | Restart the notebook kernel and re-run all cells from the top |
| `ros2-env` kernel not in VS Code | Run `pip install ipykernel && python3 -m ipykernel install --user --name ros2-env --display-name "ros2-env"` inside the venv |
| colcon build fails with venv errors | Ensure `modules/ros2-env/COLCON_IGNORE` file exists |
