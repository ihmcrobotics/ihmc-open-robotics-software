package us.ihmc.tools.compression;

import java.io.IOException;

import org.xerial.snappy.Snappy;

/**
 * This class is an abstraction of the actual Snappy implementation.
 * You can extend this class and override its methods on other platforms (e.g. Android), in order to make it work with other libraries.
 */
public class SnappyLibrary
{
   public int maxCompressedLength(int sourceLength)
   {
      return Snappy.maxCompressedLength(sourceLength);
   }

   public int compress(byte[] input, int inputOffset, int inputLength, byte[] output, int outputOffset) throws IOException
   {
      return Snappy.compress(input, inputOffset, inputLength, output, outputOffset);
   }

   public int uncompress(byte[] input, int inputOffset, int inputLength, byte[] output, int outputOffset) throws IOException
   {
      return Snappy.uncompress(input, inputOffset, inputLength, output, outputOffset);
   }
}
