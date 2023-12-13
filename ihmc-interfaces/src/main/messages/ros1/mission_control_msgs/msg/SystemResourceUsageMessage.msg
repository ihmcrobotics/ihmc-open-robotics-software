
string uptime

# ======== System memory statistics ========
float32 memory_used

float32 memory_total

# ======== CPU statistics ========
int32 cpu_count

# [n] : usage for nth CPU
float32[] cpu_usages

# [n] : temperature for nth CPU
float32[] cpu_temps

# ======== Network statistics ========
int32 iface_count

# [n] : name for nth iface
string[] iface_names

# [n] : current rx (kbps) for nth iface
float32[] iface_rx_kbps

# [n] : current tx (kbps) for nth iface
float32[] iface_tx_kbps

# ======== NVIDIA GPU statistics ========
int32 nvidia_gpu_count

# [n] : model for nth GPU
string[] nvidia_gpu_models

# [n] : memory used for nth GPU
float32[] nvidia_gpu_memory_used

# [n] : total memory for nth GPU
float32[] nvidia_gpu_memory_total

# [n] : utilization for nth GPU
float32[] nvidia_gpu_utilization

# [n] : die temperature for nth GPU
float32[] nvidia_gpu_temps


