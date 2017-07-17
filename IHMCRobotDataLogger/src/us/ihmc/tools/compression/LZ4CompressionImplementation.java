package us.ihmc.tools.compression;

import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;

import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import net.jpountz.util.Native;

/**
 * Helper class to easily switch compression algorithms . 
 * 
 * @author Jesper Smith
 *
 */
public class LZ4CompressionImplementation implements CompressionImplementation
{
   private final boolean nativeInstance;
   private final LZ4Compressor compressor;
   private final LZ4FastDecompressor decompressor;

   public LZ4CompressionImplementation()
   {
      LZ4Factory instance;
      boolean nativeInstance;
      if (Native.isLoaded() || Native.class.getClassLoader() == ClassLoader.getSystemClassLoader())
      {
         try
         {
            instance = LZ4Factory.nativeInstance();
            nativeInstance = true;
         }
         catch (Throwable t)
         {
            instance = LZ4Factory.fastestJavaInstance();
            nativeInstance = false;
         }
      }
      else
      {
         instance = LZ4Factory.fastestJavaInstance();
         nativeInstance = false;
      }
      if (!nativeInstance)
      {
         System.err.println("Cannot load native LZ4 implementation, falling back to slower Java implementation.");
      }

      this.nativeInstance = nativeInstance;

      compressor = instance.fastCompressor();
      decompressor = instance.fastDecompressor();
   }

   @Override
   public boolean supportsDirectOutput()
   {
      return nativeInstance;
   }

   @Override
   public int compress(ByteBuffer src, ByteBuffer target)
   {
      int targetPosition = target.position();
      compressor.compress(src, target);
      return target.position() - targetPosition;
   }

   @Override
   public void decompress(ByteBuffer src, ByteBuffer target, int decompressedLength)
   {
      if (target.position() + decompressedLength > target.limit())
      {
         throw new BufferOverflowException();
      }

      int read = decompressor.decompress(src, src.position(), target, target.position(), decompressedLength);
      target.position(target.position() + decompressedLength);
      src.position(src.position() + read);
   }

   @Override
   public int maxCompressedLength(int uncompressedLength)
   {
      return compressor.maxCompressedLength(uncompressedLength);
   }

   @Override
   public int minimumDecompressedLength(int compressedLength)
   {
      double y = ((long) (compressedLength - 16)) * 255;
      return (int) Math.round(y / 256);
   }
   

}
