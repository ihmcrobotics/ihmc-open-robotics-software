package us.ihmc.tools.compression;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class SnappyUtilsTest
{
      private final Random rand = new Random(98753244356L);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testDirectByteBuffers() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocateDirect(elements * 4 + inOffset);
         ByteBuffer out = ByteBuffer.allocateDirect(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset);
         ByteBuffer decompress = ByteBuffer.allocateDirect(elements * 4 + decompressOffset);
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testDirectInput() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocateDirect(elements * 4 + inOffset);
         ByteBuffer out = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset);
         ByteBuffer decompress = ByteBuffer.allocate(elements * 4 + decompressOffset);
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testDirectOutput() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocate(elements * 4 + inOffset);
         ByteBuffer out = ByteBuffer.allocateDirect(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset);
         ByteBuffer decompress = ByteBuffer.allocate(elements * 4 + decompressOffset);
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testDirectCompressed() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocate(elements * 4 + inOffset);
         ByteBuffer out = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset);
         ByteBuffer decompress = ByteBuffer.allocateDirect(elements * 4 + decompressOffset);
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testHeap() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocate(elements * 4 + inOffset);
         ByteBuffer out = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset);
         ByteBuffer decompress = ByteBuffer.allocate(elements * 4 + decompressOffset);
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testSlice() throws IOException
      {
         int elements = 128 + rand.nextInt(128);
         int inOffset = rand.nextInt(128);
         int outOffset = rand.nextInt(128);
         int decompressOffset = rand.nextInt(128);
         
         int inSlice = rand.nextInt(128);
         int outSlice = rand.nextInt(128);
         int decompressSlice = rand.nextInt(128);
         
         ByteBuffer in = ByteBuffer.allocate(elements * 4 + inOffset + inSlice);
         ByteBuffer out = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(in.remaining()) + outOffset + outSlice);
         ByteBuffer decompress = ByteBuffer.allocate(elements * 4 + decompressOffset + decompressSlice);
         
         in.position(inSlice);
         in = in.slice();
         out.position(outSlice);
         out = out.slice();
         decompress.position(decompressSlice);
         decompress = decompress.slice();
         
         testCompression(elements, in, inOffset, out, outOffset, decompress, decompressOffset);
      }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
      public void testCompressionRatio() throws IOException
      {
         int elements = 1024;
         ByteBuffer random = ByteBuffer.allocate(elements);
         ByteBuffer tens = ByteBuffer.allocate(elements);
         
         ByteBuffer randomOut = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(elements));
         ByteBuffer tensOut = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(elements));
         
         for(int i = 0; i < elements/4; i++)
         {
            random.putInt(rand.nextInt());
            tens.putInt(10);
         }
         random.flip();
         tens.flip();
         
         SnappyUtils.compress(random, randomOut);
         SnappyUtils.compress(tens, tensOut);
         
         
         assertTrue(tensOut.position() < randomOut.position());
      }

      private void testCompression(int elements, ByteBuffer in, int inOffset, ByteBuffer out, int outOffset, ByteBuffer decompress, int decompressOffset) throws IOException
      {
         in.position(inOffset);
         out.position(outOffset);
         decompress.position(decompressOffset);

         for(int i = 0; i < elements; i++)
         {
            in.putInt(rand.nextInt());
         } 
         
         in.flip();
         in.position(inOffset);
         
         SnappyUtils.compress(in, out);
         assertEquals(0, in.remaining());
         out.flip();
         out.position(outOffset);
         
         SnappyUtils.uncompress(out, decompress);
         
         in.position(inOffset);
         decompress.flip();
         decompress.position(decompressOffset);
         
         assertEquals(elements * 4, decompress.remaining());
         for(int i = 0; i < elements; i++)
         {
            assertEquals(in.getInt(), decompress.getInt());
         }
      }

}
