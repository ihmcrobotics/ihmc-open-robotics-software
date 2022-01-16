package us.ihmc.perception;

import com.google.common.io.Resources;
import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.SizeTPointer;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;

import static org.bytedeco.opencl.global.OpenCL.CL_SUCCESS;

public class OpenCLTools
{
   private static final int stringSizeByteLimit = Conversions.megabytesToBytes(2);
   private static ThreadLocal<CharPointer> stringPointer = ThreadLocal.withInitial(() -> new CharPointer(stringSizeByteLimit));
   private static ThreadLocal<SizeTPointer> resultingStringLengthPointer = ThreadLocal.withInitial(() -> new SizeTPointer(1));

   public static String readFile(Path file)
   {
      return ExceptionTools.handle(() -> Resources.toString(ResourceTools.getResourceSystem(file), StandardCharsets.UTF_8),
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
         LogTools.error(1, "OpenCL error code: " + returnCode);
      }
   }
}
