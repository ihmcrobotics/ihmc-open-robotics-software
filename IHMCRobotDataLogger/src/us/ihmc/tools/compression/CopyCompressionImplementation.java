package us.ihmc.tools.compression;

import java.nio.ByteBuffer;

public class CopyCompressionImplementation implements CompressionImplementation
{

   @Override
   public boolean supportsDirectOutput()
   {
      return true;
   }

   @Override
   public int compress(ByteBuffer src, ByteBuffer target)
   {
      int remaining = src.remaining();
      target.put(src);
      return remaining;
   }

   @Override
   public void decompress(ByteBuffer src, ByteBuffer target, int decompressedLength)
   {
      int currentLimit = src.limit();
      src.limit(src.position() + decompressedLength);
      target.put(src);
      src.limit(currentLimit);
   }

   @Override
   public int maxCompressedLength(int uncompressedLength)
   {
      return uncompressedLength;
   }

   @Override
   public int minimumDecompressedLength(int compressedLength)
   {
      return compressedLength;
   }
   
}
