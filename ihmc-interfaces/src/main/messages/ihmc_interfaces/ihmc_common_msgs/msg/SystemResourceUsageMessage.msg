# This is part of the IHMC Common message package.

# ======== System memory statistics ========
float32 memory_used
float32 total_memory

# ======== CPU statistics ========
int32 cpu_count
#   [n] : usage for nth CPU
float32[] cpu_usages

# ======== Network statistics ========
float32 net_bytes_in
float32 net_bytes_out

# ======== NVIDIA GPU statistics ========
int32 nvidia_gpu_count
#   [n] : memory used for nth GPU
float32[] nvidia_gpu_memory_used
#   [n] : total memory for nth GPU
float32[] nvidia_gpu_total_memory
#   [n] : utilization for nth GPU
float32[] nvidia_gpu_utilization
