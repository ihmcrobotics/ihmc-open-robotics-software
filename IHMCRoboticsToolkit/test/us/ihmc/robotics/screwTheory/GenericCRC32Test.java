package us.ihmc.robotics.screwTheory;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.BitSet;
import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * Created by nathan on 8/12/15.
 */
public class GenericCRC32Test
{
   private final int CRC_32_POLYNOMIAL = 79764919;
   private final ByteBuffer CRC_32_BYTE_BUFFER = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
   private BitSet CRC_32_BITSET;

   @Before
   public void initialize()
   {
      CRC_32_BYTE_BUFFER.putInt(CRC_32_POLYNOMIAL);
      CRC_32_BYTE_BUFFER.rewind();
      CRC_32_BITSET = BitSet.valueOf(CRC_32_BYTE_BUFFER);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUpdateLong()
   {
      Random random = new Random(4271L);
      GenericCRC32 genericCRC32 = new GenericCRC32();
      genericCRC32.reset();
      ByteBuffer buffer = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);

      genericCRC32.update(CRC_32_POLYNOMIAL);
      System.out.println(genericCRC32.getValue());

   }

   private BitSet calculateCRCForLittleEndianBitSet(BitSet bitSet, BitSet crc)
   {
      int bitSetLength = bitSet.length();
      BitSet resultBitSet = createResultBitSet(bitSet, crc.length());

      for(int i = bitSetLength - 1; i >= 0; i--)
      {
         if(bitSet.get(i))
         {
            BitSet temp2 = resultBitSet.get(i, i + crc.length());
            temp2.xor(crc);
            for(int j = crc.length() - 1; j >= 0; j--)
            {
               if(temp2.get(j))
                  resultBitSet.set(i + j);
               else
                  resultBitSet.clear(i + j);
            }

            bitSet = resultBitSet.get(crc.length() - 1, crc.length() + bitSetLength);
         }
      }

      return resultBitSet.get(0, crc.length());
   }

   private BitSet createResultBitSet(BitSet sourceBitSet, int crcLength)
   {
      BitSet ret = new BitSet();

      for(int i = 0; i < sourceBitSet.length(); i++)
      {
         if(sourceBitSet.get(i))
            ret.set(i + crcLength - 1);
      }

      return ret;
   }

   private void printBitSet(BitSet bitSet)
   {
      String ret = "";

      for(int i = 0; i < bitSet.length(); i++)
      {
         ret += bitSet.get(i) ? "1" : "0";
      }

      System.out.println(ret);
   }
}
