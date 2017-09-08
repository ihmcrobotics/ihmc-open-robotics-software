package us.ihmc.tools.io;

import java.io.ByteArrayInputStream;
import java.io.IOException;

public class ResettableByteArrayInputStream extends ByteArrayInputStream
{
   private static final byte[] EMPTY_ARRAY = new byte[0];

   public ResettableByteArrayInputStream()
   {
      this(EMPTY_ARRAY, 0);
   }

   public ResettableByteArrayInputStream(byte buffer[], int length)
   {
      super(buffer, 0, length);
   }

   public ResettableByteArrayInputStream(byte buffer[], int offset, int length)
   {
      super(buffer, offset, length);
   }

   public byte[] getBytes()
   {
      return buf;
   }

   public int getCount()
   {
      return count;
   }

   @Override
   public void close() throws IOException
   {
      reset();
   }

   public void setBuffer(byte[] buffer, int length)
   {
      this.buf = buffer;
      this.pos = 0;
      this.count = length;
   }
}
