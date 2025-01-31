// Here we haven't declared the (extern "C") so this should not load correctly
__global__ void kernel_not_declared_correctly()
{
    // Leave this kernel empty because its not meant to load correctly
    // This is because we didn't declare it with (extern "C")
    printf("Hello World");
}

// This kernel has the correct code to run properly.
// Here we return a value to the CPU at the end of the kernel
extern "C"
__global__ void pass_in_variable(int value,
                                 float* valueBeingReturned)
{
    printf("Value passed in is: %d\n", value);
    *valueBeingReturned = value;
}

// Simple kernel that only takes in an int for testing
extern "C"
__global__ void pass_in_int(int value)
{
    printf("Only an int being passed to this kernel");
}