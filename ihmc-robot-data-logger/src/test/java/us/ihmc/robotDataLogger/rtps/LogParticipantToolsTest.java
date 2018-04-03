package us.ihmc.robotDataLogger.rtps;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotDataLogger.rtps.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.rtps.DataProducerParticipant;
import us.ihmc.robotDataLogger.rtps.LogParticipantTools;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;



public class LogParticipantToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1200000)
   public void testSegmentation()
   {
      CompressionImplementation compressor = CompressionImplementationFactory.instance();
      int maxJointStates = (DataProducerParticipant.getMaximumSynchronousPacketSize() - CustomLogDataPublisherType.getTypeSize(0, 0)) / 8 - 1;
      for(int vars = 1; vars < 100000; vars+=11)
      {
         for(int states = 0; states < maxJointStates; states++)
         {
            int sizes[] = LogParticipantTools.calculateLogSegmentSizes(vars, states);
            
            int totalSize = 0;
            for(int i = 0; i < sizes.length; i++)
            {
               int size = sizes[i];
               int packetStates = i == 0 ? states : 0;
               assertTrue("Packet too big", CustomLogDataPublisherType.getTypeSize(compressor.maxCompressedLength(size), packetStates) < DataProducerParticipant.getMaximumSynchronousPacketSize());
               totalSize += size;
            }
            assertEquals(vars, totalSize);
         }
      }
   }
}
