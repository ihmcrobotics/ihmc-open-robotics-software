package us.ihmc.perception;

import com.google.common.io.Resources;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.charset.StandardCharsets;
import java.nio.file.Path;

public class OpenCLTools
{
   public static String readFile(Path file)
   {
      return ExceptionTools.handle(() -> Resources.toString(ResourceTools.getResourceSystem(file), StandardCharsets.UTF_8),
                                   DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
