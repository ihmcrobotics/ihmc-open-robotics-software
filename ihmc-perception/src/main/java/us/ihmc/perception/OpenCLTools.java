package us.ihmc.perception;

import com.google.common.io.Resources;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl.*;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.bytedeco.opencl.global.OpenCL.*;
import static org.bytedeco.opencl.global.OpenCL.clCreateContext;

public class OpenCLTools
{
   public static void createOpenCLContext()
   {
      _cl_platform_id platform_id = new _cl_platform_id(null);
      _cl_device_id device_id = new _cl_device_id(null);
      _cl_context context = new _cl_context(null);
      _cl_command_queue command_queue = new _cl_command_queue(null);
      _cl_mem memobj = new _cl_mem(null);
      _cl_program program = new _cl_program(null);
      _cl_kernel kernel = new _cl_kernel(null);
      IntPointer ret_num_devices = new IntPointer(1);
      IntPointer ret_num_platforms = new IntPointer(1);
      IntPointer ret_pointer = new IntPointer(1);
      int ret;

      /* Get platform/device information */
      ret = clGetPlatformIDs(1, platform_id, ret_num_platforms);
      ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, device_id, ret_num_devices);

      /* Create OpenCL Context */
      context = clCreateContext(null, 1, device_id, null, null, ret_pointer);

      /* Create Command Queue */
      IntPointer properties = new IntPointer(new int[] {0});
      command_queue = clCreateCommandQueueWithProperties(context, device_id, properties, ret_pointer);
   }

   public static String readFile(Path file)
   {
      return ExceptionTools.handle(() -> Resources.toString(ResourceTools.getResourceSystem(file), StandardCharsets.UTF_8),
                                   DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
