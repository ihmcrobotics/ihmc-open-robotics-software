package us.ihmc.gdx.ui.missionControl;

import imgui.type.ImString;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.Arrays;

public class SSHJInputStream
{
   private InputStream inputStream;
   private Charset charset;
   private ByteBuffer inputByteBuffer;

   public void resize(int bufferSize)
   {
      inputByteBuffer = ByteBuffer.allocate(bufferSize);
   }

   public void setInputStream(InputStream inputStream, Charset charset)
   {
      this.inputStream = inputStream;
      this.charset = charset;
   }

   public void updateConsoleText(ImString consoleText)
   {
      if (inputStream != null)
      {
         inputByteBuffer.rewind();

         int availableBytes = ExceptionTools.handle(inputStream::available, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         for (int i = 0; i < availableBytes; i++)
         {
            int read = ExceptionTools.handle(() -> inputStream.read(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            inputByteBuffer.put((byte) read);
         }

         String newPart = new String(Arrays.copyOf(inputByteBuffer.array(), inputByteBuffer.position()), charset);
         consoleText.set(consoleText.get() + newPart);
         System.out.print(newPart);
      }
   }
}
