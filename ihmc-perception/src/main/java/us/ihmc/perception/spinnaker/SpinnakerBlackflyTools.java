package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.nio.charset.StandardCharsets;
import java.util.function.Supplier;

import static org.bytedeco.spinnaker.global.Spinnaker_C.spinErrorGetLastFullMessage;

public final class SpinnakerBlackflyTools
{
   public static Spinnaker_C.spinError printOnError(Spinnaker_C.spinError error, String errorMessage)
   {
      if (error.value != Spinnaker_C.spinError.SPINNAKER_ERR_SUCCESS.value)
      {
         long size = 1000;
         BytePointer fullMessage = new BytePointer(size);
         SizeTPointer errorMessageSize = new SizeTPointer(1L);
         errorMessageSize.put(size);
         spinErrorGetLastFullMessage(fullMessage, errorMessageSize);
         StringBuilder fullMessageString = new StringBuilder();
         long errorMessageSizeInt = errorMessageSize.get();
         for (int i = 0; i < errorMessageSizeInt + 1; i++)
         {
            fullMessageString.append(new String(new byte[] {fullMessage.get(i)}, StandardCharsets.UTF_8));
         }

         Supplier<String> message = StringTools.format("Error code: {}: {}: {}: {}", error.value, error.toString(), fullMessageString.toString(), errorMessage);
         LogTools.error(message);
      }

      return error;
   }
}
