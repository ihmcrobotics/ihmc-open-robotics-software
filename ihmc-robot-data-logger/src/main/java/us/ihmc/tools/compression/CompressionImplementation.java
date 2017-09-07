package us.ihmc.tools.compression;

import java.nio.ByteBuffer;

public interface CompressionImplementation
{
   /**
    * Query if the compression algorithm supports direct output buffers with java input buffers
    * 
    * @return true if using direct buffers
    */
   public boolean supportsDirectOutput();
   
   /**
    * Compress src into target
    * 
    * @param src Non compressed buffer
    * @param target Compressed data buffer
    * 
    * @return compressed data size
    */
   public int compress(ByteBuffer src, ByteBuffer target);
   
   /**
    * Decompress target into src
    * 
    * @param src Compressed data buffer
    * @param target Non compressed data buffer
    * @param decompressedLength Length the decompressed data will be
    */
   public void decompress(ByteBuffer src, ByteBuffer target, int decompressedLength);

   /**
    * Get the maximum size of the compressed data buffer
    * @param uncompressedLength
    * @return Maximum size of the compressed data
    */
   public int maxCompressedLength(int uncompressedLength);
   
   
   /**
    * Get the minimum size of the decompressed data
    * 
    * Due to integer rounding, the following relation holds
    * length - 1 >= minimumDeCompressedLength(maxCompressedLength(int length)) <= length
    * 
    * @return Minimum size of decompressed data
    */
   public int minimumDecompressedLength(int compressedLength);
}
