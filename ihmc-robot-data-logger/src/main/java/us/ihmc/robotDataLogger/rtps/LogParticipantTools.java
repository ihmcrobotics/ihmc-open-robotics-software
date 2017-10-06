package us.ihmc.robotDataLogger.rtps;

import java.util.Arrays;

import us.ihmc.pubsub.common.Guid;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;

public class LogParticipantTools
{
   public static String createGuidString(Guid guid)
   {
      StringBuilder guidBuilder = new StringBuilder();
      guidBuilder.append("{");
      for (byte val : guid.getGuidPrefix().getValue())
      {
         guidBuilder.append(String.format("%02x", val));
      }
      //      guidBuilder.append("-");
      //      for(byte val : guid.getEntity().getValue())
      //      {
      //         guidBuilder.append(String.format("%02x", val));
      //      }
      guidBuilder.append("}");
      return guidBuilder.toString();

   }
   
   public static int calculateMaximumNumberOfVariables(int variables, int jointStates)
   {
      int[] segmentSizes = calculateLogSegmentSizes(variables, jointStates);
      int max = 0;
      for(int size : segmentSizes)
      {
         if(size > max)
         {
            max = size;
         }
      }
      return max;
   }
   
   /**
    * Calculate the number of variables in each segment
    * 
    * @param variables total number of variables
    * @param jointStates total number of joint states
    * @return array of variables in each segment
    */
   public static int[] calculateLogSegmentSizes(int variables, int jointStates)
   {
      CompressionImplementation compressor = CompressionImplementationFactory.instance();
      int maxCompressedSize = compressor.maxCompressedLength(variables * 8);
      int maxSize = CustomLogDataPublisherType.getTypeSize(maxCompressedSize, jointStates);

      if (maxSize > DataProducerParticipant.getMaximumSynchronousPacketSize())
      {
         // Space available in bytes for variables
         int remaining = DataProducerParticipant.getMaximumSynchronousPacketSize() - CustomLogDataPublisherType.getTypeSize(0, 0);

         // Total variables in packet. Adding the number of uncompressed joint states gives us a slightly larger estimate of how much space we need.
         int totalVariables = variables + jointStates;

         // Variables we can store in each packet, rounding down
         int variablesPerPacket = compressor.minimumDecompressedLength(remaining) / 8;

         // Number of packets needed to store all variables, including joint states. Rounding up.
         int packets = (totalVariables - 1) / variablesPerPacket + 1;

         // Variables per packet, rounding up.
         int averageVariablesPerPacket = (totalVariables - 1) / packets + 1;

         int[] variableCounts = new int[packets];
         // If there are more joint states than variables per packet put all variables in subsequent packets 
         if (averageVariablesPerPacket <= jointStates)
         {
            variableCounts[0] = 0;
            int variablesPerResultingPacket = (variables - 1) / (packets - 1) + 1;
            Arrays.fill(variableCounts, 1, variableCounts.length, variablesPerResultingPacket);
         }
         else // Average it out
         {
            variableCounts[0] = averageVariablesPerPacket - jointStates;
            int variablesPerResultingPacket = (variables - variableCounts[0] - 1) / (packets - 1) + 1;
            Arrays.fill(variableCounts, 1, variableCounts.length, variablesPerResultingPacket);
         }

         // Account for rounding errors, make sure we have enough space
         int sum = 0;
         for (int val : variableCounts)
         {
            sum += val;
         }
         variableCounts[variableCounts.length - 1] += variables - sum;

         return variableCounts;
      }
      else
      {
         return new int[] {variables};
      }
   }
   
   public static int[] calculateOffsets(int[] segmentSizes)
   {
      int[] offsets = new int[segmentSizes.length];
      int currentOffset = 0;
      for(int i = 0; i < segmentSizes.length; i++)
      {
         offsets[i] = currentOffset;
         currentOffset += segmentSizes[i];
      }
      
      return offsets;
      
   }

}
