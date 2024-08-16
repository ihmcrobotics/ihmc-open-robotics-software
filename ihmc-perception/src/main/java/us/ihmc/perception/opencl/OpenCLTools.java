package us.ihmc.perception.opencl;

import com.google.common.io.Resources;
import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencl._cl_device_id;
import org.bytedeco.opencl._cl_platform_id;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;

import static org.bytedeco.opencl.global.OpenCL.*;

public class OpenCLTools
{
   private static final int stringSizeByteLimit = Conversions.megabytesToBytes(2);
   private static final ThreadLocal<CharPointer> stringPointer = ThreadLocal.withInitial(() -> new CharPointer(stringSizeByteLimit));
   private static final ThreadLocal<SizeTPointer> resultingStringLengthPointer = ThreadLocal.withInitial(() -> new SizeTPointer(1));

   public static String readFile(Path path)
   {
      return ExceptionTools.handle(() -> Resources.toString(ResourceTools.getResourceSystem(path), StandardCharsets.UTF_8),
                                   DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static String readString(OpenCLStringProvider stringProvider)
   {
      stringProvider.read(stringSizeByteLimit, stringPointer.get(), resultingStringLengthPointer.get());
      ByteBuffer byteBuffer = stringPointer.get().asByteBuffer();
      int infoStringLength = (int) resultingStringLengthPointer.get().get();
      byte[] bytes = new byte[infoStringLength];
      byteBuffer.get(bytes, 0, infoStringLength);
      return new String(bytes, StandardCharsets.UTF_8).trim();
   }

   public static void checkReturnCode(int returnCode)
   {
      if (returnCode != CL_SUCCESS) // duplicated to reduce stack trace height
      {
         throw new RuntimeException("OpenCL error code: (" + returnCode + ") -> " + getReturnCodeString(returnCode));
//         LogTools.error( "OpenCL error code: ({}) -> {}", returnCode, getReturnCodeString(returnCode));
      }
   }

   public static void checkReturnCode(IntPointer returnCodePointer)
   {
      checkReturnCode(returnCodePointer.get());
   }

   public static String readPlatformInfoParameter(_cl_platform_id platformId, int i, int parameterName)
   {
      return OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
      {
         checkReturnCode(clGetPlatformInfo(platformId.position(i).getPointer(),
                                           parameterName,
                                           stringSizeByteLimit,
                                           stringPointer,
                                           resultingStringLengthPointer));
      });
   }

   public static String readDeviceInfoParameter(_cl_device_id deviceId, int i, int parameterName)
   {
      return OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
      {
         checkReturnCode(clGetDeviceInfo(deviceId.position(i).getPointer(),
                                         parameterName,
                                         stringSizeByteLimit,
                                         stringPointer,
                                         resultingStringLengthPointer));
      });
   }

   public static String getReturnCodeString(int returnCode)
   {
      return switch (returnCode)
      {
         case OpenCL.CL_SUCCESS -> "CL_SUCCESS";
         case OpenCL.CL_DEVICE_NOT_FOUND -> "CL_DEVICE_NOT_FOUND";
         case OpenCL.CL_DEVICE_NOT_AVAILABLE -> "CL_DEVICE_NOT_AVAILABLE";
         case OpenCL.CL_COMPILER_NOT_AVAILABLE -> "CL_COMPILER_NOT_AVAILABLE";
         case OpenCL.CL_MEM_OBJECT_ALLOCATION_FAILURE -> "CL_MEM_OBJECT_ALLOCATION_FAILURE";
         case OpenCL.CL_OUT_OF_RESOURCES -> "CL_OUT_OF_RESOURCES";
         case OpenCL.CL_OUT_OF_HOST_MEMORY -> "CL_OUT_OF_HOST_MEMORY";
         case OpenCL.CL_PROFILING_INFO_NOT_AVAILABLE -> "CL_PROFILING_INFO_NOT_AVAILABLE";
         case OpenCL.CL_MEM_COPY_OVERLAP -> "CL_MEM_COPY_OVERLAP";
         case OpenCL.CL_IMAGE_FORMAT_MISMATCH -> "CL_IMAGE_FORMAT_MISMATCH";
         case OpenCL.CL_IMAGE_FORMAT_NOT_SUPPORTED -> "CL_IMAGE_FORMAT_NOT_SUPPORTED";
         case OpenCL.CL_BUILD_PROGRAM_FAILURE -> "CL_BUILD_PROGRAM_FAILURE";
         case OpenCL.CL_MAP_FAILURE -> "CL_MAP_FAILURE";
         case OpenCL.CL_MISALIGNED_SUB_BUFFER_OFFSET -> "CL_MISALIGNED_SUB_BUFFER_OFFSET";
         case OpenCL.CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST -> "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
         case OpenCL.CL_COMPILE_PROGRAM_FAILURE -> "CL_COMPILE_PROGRAM_FAILURE";
         case OpenCL.CL_LINKER_NOT_AVAILABLE -> "CL_LINKER_NOT_AVAILABLE";
         case OpenCL.CL_LINK_PROGRAM_FAILURE -> "CL_LINK_PROGRAM_FAILURE";
         case OpenCL.CL_DEVICE_PARTITION_FAILED -> "CL_DEVICE_PARTITION_FAILED";
         case OpenCL.CL_KERNEL_ARG_INFO_NOT_AVAILABLE -> "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";
         case OpenCL.CL_INVALID_VALUE -> "CL_INVALID_VALUE";
         case OpenCL.CL_INVALID_DEVICE_TYPE -> "CL_INVALID_DEVICE_TYPE";
         case OpenCL.CL_INVALID_PLATFORM -> "CL_INVALID_PLATFORM";
         case OpenCL.CL_INVALID_DEVICE -> "CL_INVALID_DEVICE";
         case OpenCL.CL_INVALID_CONTEXT -> "CL_INVALID_CONTEXT";
         case OpenCL.CL_INVALID_QUEUE_PROPERTIES -> "CL_INVALID_QUEUE_PROPERTIES";
         case OpenCL.CL_INVALID_COMMAND_QUEUE -> "CL_INVALID_COMMAND_QUEUE";
         case OpenCL.CL_INVALID_HOST_PTR -> "CL_INVALID_HOST_PTR";
         case OpenCL.CL_INVALID_MEM_OBJECT -> "CL_INVALID_MEM_OBJECT";
         case OpenCL.CL_INVALID_IMAGE_FORMAT_DESCRIPTOR -> "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
         case OpenCL.CL_INVALID_IMAGE_SIZE -> "CL_INVALID_IMAGE_SIZE";
         case OpenCL.CL_INVALID_SAMPLER -> "CL_INVALID_SAMPLER";
         case OpenCL.CL_INVALID_BINARY -> "CL_INVALID_BINARY";
         case OpenCL.CL_INVALID_BUILD_OPTIONS -> "CL_INVALID_BUILD_OPTIONS";
         case OpenCL.CL_INVALID_PROGRAM -> "CL_INVALID_PROGRAM";
         case OpenCL.CL_INVALID_PROGRAM_EXECUTABLE -> "CL_INVALID_PROGRAM_EXECUTABLE";
         case OpenCL.CL_INVALID_KERNEL_NAME -> "CL_INVALID_KERNEL_NAME";
         case OpenCL.CL_INVALID_KERNEL_DEFINITION -> "CL_INVALID_KERNEL_DEFINITION";
         case OpenCL.CL_INVALID_KERNEL -> "CL_INVALID_KERNEL";
         case OpenCL.CL_INVALID_ARG_INDEX -> "CL_INVALID_ARG_INDEX";
         case OpenCL.CL_INVALID_ARG_VALUE -> "CL_INVALID_ARG_VALUE";
         case OpenCL.CL_INVALID_ARG_SIZE -> "CL_INVALID_ARG_SIZE";
         case OpenCL.CL_INVALID_KERNEL_ARGS -> "CL_INVALID_KERNEL_ARGS";
         case OpenCL.CL_INVALID_WORK_DIMENSION -> "CL_INVALID_WORK_DIMENSION";
         case OpenCL.CL_INVALID_WORK_GROUP_SIZE -> "CL_INVALID_WORK_GROUP_SIZE";
         case OpenCL.CL_INVALID_WORK_ITEM_SIZE -> "CL_INVALID_WORK_ITEM_SIZE";
         case OpenCL.CL_INVALID_GLOBAL_OFFSET -> "CL_INVALID_GLOBAL_OFFSET";
         case OpenCL.CL_INVALID_EVENT_WAIT_LIST -> "CL_INVALID_EVENT_WAIT_LIST";
         case OpenCL.CL_INVALID_EVENT -> "CL_INVALID_EVENT";
         case OpenCL.CL_INVALID_OPERATION -> "CL_INVALID_OPERATION";
         case OpenCL.CL_INVALID_GL_OBJECT -> "CL_INVALID_GL_OBJECT";
         case OpenCL.CL_INVALID_BUFFER_SIZE -> "CL_INVALID_BUFFER_SIZE";
         case OpenCL.CL_INVALID_MIP_LEVEL -> "CL_INVALID_MIP_LEVEL";
         case OpenCL.CL_INVALID_GLOBAL_WORK_SIZE -> "CL_INVALID_GLOBAL_WORK_SIZE";
         case OpenCL.CL_INVALID_PROPERTY -> "CL_INVALID_PROPERTY";
         case OpenCL.CL_INVALID_IMAGE_DESCRIPTOR -> "CL_INVALID_IMAGE_DESCRIPTOR";
         case OpenCL.CL_INVALID_COMPILER_OPTIONS -> "CL_INVALID_COMPILER_OPTIONS";
         case OpenCL.CL_INVALID_LINKER_OPTIONS -> "CL_INVALID_LINKER_OPTIONS";
         case OpenCL.CL_INVALID_DEVICE_PARTITION_COUNT -> "CL_INVALID_DEVICE_PARTITION_COUNT";
         case OpenCL.CL_INVALID_PIPE_SIZE -> "CL_INVALID_PIPE_SIZE";
         case OpenCL.CL_INVALID_DEVICE_QUEUE -> "CL_INVALID_DEVICE_QUEUE";
         case OpenCL.CL_INVALID_SPEC_ID -> "CL_INVALID_SPEC_ID";
         case OpenCL.CL_MAX_SIZE_RESTRICTION_EXCEEDED -> "CL_MAX_SIZE_RESTRICTION_EXCEEDED";
         default -> "Code not found";
      };
   }
}
