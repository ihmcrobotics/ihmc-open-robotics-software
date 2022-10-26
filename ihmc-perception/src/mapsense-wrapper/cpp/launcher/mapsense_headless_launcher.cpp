// #include "mapsense_headless_launcher.h"


// MapsenseHeadlessLauncher::MapsenseHeadlessLauncher(int argc, char **argv)
// {
//    MapsenseExternal mapsense;

//    float bufferA[] = {1.2, 2.3, 3.4, 4.5, 5.6, 6.7};
//    float bufferB[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
//    float bufferOutput[6] = {0};

//    mapsense.testOpenCLParallelAdd(bufferA, bufferB, bufferOutput, 6);

//    for(int i = 0; i<6; i++)
//    {
//       printf("BufferOutput[%d]: %.3lf\n", i, bufferOutput[i]);
//    }

// }

// int main(int argc, char **argv)
// {
//    // MapsenseHeadlessLauncher mapsense(argc, argv);

   

// }





int main(int argc, char* argv[])
{

	

	/*Step 7: Initial input,output for the host and create memory objects for the kernel*/
	const char* input = "GdkknVnqkc";
	size_t strlength = strlen(input);
	cout << "input string:" << endl;
	cout << input << endl;
	char *output = (char*)malloc(strlength + 1);

	cl_mem inputBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, 
                             (strlength + 1) * sizeof(char), (void *)input, NULL);
	cl_mem outputBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, 
                              (strlength + 1) * sizeof(char), NULL, NULL);

	/*Step 8: Create kernel object */
	cl_kernel kernel = clCreateKernel(program, "helloworld", NULL);

	/*Step 9: Sets Kernel arguments.*/
	status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&inputBuffer);
	status = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&outputBuffer);

	/*Step 10: Running the kernel.*/
	size_t global_work_size[1] = { strlength };
	status = clEnqueueNDRangeKernel(commandQueue, kernel, 1, NULL, 
                                        global_work_size, NULL, 0, NULL, NULL);

	/*Step 11: Read the cout put back to host memory.*/
	status = clEnqueueReadBuffer(commandQueue, outputBuffer, CL_TRUE, 0, 
                 strlength * sizeof(char), output, 0, NULL, NULL);

	output[strlength] = '\0'; //Add the terminal character to the end of output.
	cout << "\noutput string:" << endl;
	cout << output << endl;

	/*Step 12: Clean the resources.*/
	status = clReleaseKernel(kernel); //Release kernel.
	status = clReleaseProgram(program); //Release the program object.
	status = clReleaseMemObject(inputBuffer); //Release mem object.
	status = clReleaseMemObject(outputBuffer);
	status = clReleaseCommandQueue(commandQueue); //Release  Command queue.
	status = clReleaseContext(context); //Release context.

	if (output != NULL)
	{
		free(output);
		output = NULL;
	}

	if (devices != NULL)
	{
		free(devices);
		devices = NULL;
	}

	std::cout << "Passed!\n";
	return SUCCESS;
}