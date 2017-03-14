package us.ihmc.tools.compression;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.commons.PrintTools;

public class SnappyUtils
{
   public static final boolean DISABLE_COMPRESSION = false;
   static
   {
      if (DISABLE_COMPRESSION)
      {
         PrintTools.warn(SnappyUtils.class, "DISABLE_COMPRESSION is set to true");
      }
   }

   public static SnappyLibrary snappyLibrary = new SnappyLibrary();
   public static void setLibrary(SnappyLibrary library)
   {
      snappyLibrary = library;
   }

   /**
    * Compress input ByteBuffer to output ByteBuffer using Snappy
    * 
    * @param input More efficient if HeapByteBuffer
    * @param output More efficient if HeapByteBuffer
    * @throws IOException 
    */
   public static void compress(ByteBuffer input, ByteBuffer output) throws IOException
   {
      int length = maxCompressedLength(input.remaining());
      if (output.remaining() < length)
      {
         throw new IllegalArgumentException("Cannot compress to output buffer, buffer size is: " + output.remaining() + ", need " + length);
      }

      byte[] in;
      int inOffset, inLength;
      if (!input.hasArray())
      {
         inLength = input.remaining();
         in = new byte[inLength];
         input.get(in);
         inOffset = 0;
      }
      else
      {
         in = input.array();
         inOffset = input.position() + input.arrayOffset();
         inLength = input.remaining();
         input.position(input.position() + input.remaining());
      }
      byte[] out;
      int outOffset;
      if (!output.hasArray())
      {
         int outLength = maxCompressedLength(inLength);
         out = new byte[outLength];
         outOffset = 0;
      }
      else
      {
         out = output.array();
         outOffset = output.position() + output.arrayOffset();
      }
      
      int compressedSize;
      if(DISABLE_COMPRESSION)
      {
         System.arraycopy(in, inOffset, out, outOffset, inLength);
         compressedSize = inLength;
      }
      else
      {
         compressedSize = snappyLibrary.compress(in, inOffset, inLength, out, outOffset);
      }
      
      if (!output.hasArray())
      {
         output.put(out, outOffset, compressedSize);
      }
      else
      {
         output.position(output.position() + compressedSize);
      }
   }
   
   /**
    * Uncompress Snappy compressed data packet. 
    * 
    * @param input Snappy compressed data, faster if HeapByteBuffer
    * @param output Decompressed data, faster if HeapByteBuffer
    * @throws IllegalArgumentException
    * @throws IOException 
    */
   public static void uncompress(ByteBuffer input, ByteBuffer output) throws IllegalArgumentException, IOException
   {
      byte[] in;
      int inOffset, inLength;
      if(!input.hasArray())
      {
         inOffset = 0;
         inLength = input.remaining();
         in = new byte[inLength];
         input.get(in);
      }
      else
      {
         inOffset = input.position() + input.arrayOffset();
         inLength = input.remaining();
         in = input.array();
         input.position(input.position() + input.remaining());
      }
      
      byte[] out;
      int outOffset;
      if (!output.hasArray())
      {
         int outLength = output.remaining();
         out = new byte[outLength];
         outOffset = 0;
      }
      else
      {
         out = output.array();
         outOffset = output.position() + output.arrayOffset();
      }
      
      int uncompressedSize;
      if(DISABLE_COMPRESSION)
      {
         System.arraycopy(in, inOffset, out, outOffset, inLength);
         uncompressedSize = inLength;
      }
      else
      {
         uncompressedSize = snappyLibrary.uncompress(in, inOffset, inLength, out, outOffset);
      }
      
      if(!output.hasArray())
      {
         output.put(out, outOffset, uncompressedSize);
      }
      else
      {
         output.position(output.position() + uncompressedSize);
      }
   }
   
   /**
    * Get the maximum size of the compressed data
    * 
    * @param sourceLength Length of uncompressed data
    * @return Maximum size of compressed data
    */
   public static int maxCompressedLength(int sourceLength)
   {
      return snappyLibrary.maxCompressedLength(sourceLength);
   }
}
