// Simple kernel that only takes in an int for testing
extern "C"
__global__ void no_semicolon(int value)
{
    // When this is compiled this should fail because its not proper C code (missing ;)
    printf("Only an int being passed to this kernel")
}