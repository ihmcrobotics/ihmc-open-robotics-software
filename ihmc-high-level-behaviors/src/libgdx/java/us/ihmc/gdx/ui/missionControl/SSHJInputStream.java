package us.ihmc.gdx.ui.missionControl;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.InputStream;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.Arrays;
import java.util.function.Consumer;

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

   public void updateConsoleText(Consumer<String> newTextConsumer)
   {
      if (inputStream != null)
      {
         inputByteBuffer.rewind();

         int availableBytes = ExceptionTools.handle(inputStream::available, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         for (int i = 0; i < availableBytes; i++)
         {
            int read = ExceptionTools.handle(() -> inputStream.read(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            try
            {
               inputByteBuffer.put((byte) read);
            }
            catch (BufferOverflowException e)
            {
               int half = inputByteBuffer.capacity() / 2;
               System.arraycopy(inputByteBuffer.array(), half, inputByteBuffer.array(), 0, half);
               inputByteBuffer.position(half);
               inputByteBuffer.put((byte) read);
            }
         }

         if (inputByteBuffer.position() > 0)
         {
            String newPart = new String(Arrays.copyOf(inputByteBuffer.array(), inputByteBuffer.position()), charset);
            newTextConsumer.accept(newPart);
         }
      }
   }

   public int getBufferSize()
   {
      return inputByteBuffer.limit();
   }
}
