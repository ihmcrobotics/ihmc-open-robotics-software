package us.ihmc.robotDataLogger.dataBuffers;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.jointState.OneDoFJointHolder;
import us.ihmc.robotDataLogger.jointState.OneDoFState;
import us.ihmc.robotDataLogger.rtps.RTPSDataProducerParticipant;
import us.ihmc.robotDataLogger.rtps.LogParticipantTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistrySendBufferTest
{
   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 57.0)
   @Test(timeout = 280000)
   public void testYoVariables() throws IOException
   {
      Random random = new Random(23589735l);
      
      
      SerializedPayload payload = new SerializedPayload(RTPSDataProducerParticipant.getMaximumSynchronousPacketSize());
      
//      int numberOfVariables = 10000;
      for(int numberOfVariables = 1000; numberOfVariables <= 33000; numberOfVariables+= 1000)
      {
         ArrayList<JointHolder> sendJointHolders = new ArrayList<>(); 
         ArrayList<JointState> receiveJointStates = new ArrayList<>();
         
         RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
         
         int numberOfJoints = random.nextInt(4000);
         for(int j = 0; j < numberOfJoints; j++)
         {
            OneDoFJointBasics sendJoint = new RevoluteJoint("Joint" + j, elevator, new Vector3D(1, 0, 0));
            sendJoint.setQ(random.nextDouble() * Math.PI * 2.0);
            sendJoint.setQd(random.nextDouble() * Math.PI * 2.0);
            sendJointHolders.add(new OneDoFJointHolder(sendJoint));
            receiveJointStates.add(new OneDoFState("Joint" + j));
            
            
         }
         
         int numberOfJointStates = RegistrySendBufferBuilder.getNumberOfJointStates(sendJointHolders);

         // Transmit data
         CustomLogDataPublisherType publisherType = new CustomLogDataPublisherType(numberOfVariables, numberOfJointStates);

         
         long timestamp = random.nextLong();
         long uid = random.nextLong();
         
         int[] sizes = LogParticipantTools.calculateLogSegmentSizes(numberOfVariables, numberOfJointStates);
         int[] offsets = LogParticipantTools.calculateOffsets(sizes);
       
         
         YoVariableRegistry sendRegistry = new YoVariableRegistry("sendRegistry");
         for(int v = 0; v < numberOfVariables; v++)
         {
            YoLong newLong = new YoLong("var" + v, sendRegistry);
            newLong.set(random.nextLong());
         }
         
         
         // Receive data
         CustomLogDataSubscriberType subscriberType = new CustomLogDataSubscriberType(LogParticipantTools.calculateMaximumNumberOfVariables(numberOfVariables, numberOfJointStates), numberOfJointStates);
         YoVariableRegistry receiveRegistry = new YoVariableRegistry("receiveRegistry");
         for(int v = 0; v < numberOfVariables; v++)
         {
            new YoLong("var" + v, receiveRegistry);
         }
         RegistryDecompressor registryDecompressor = new RegistryDecompressor(receiveRegistry.getAllVariables(), receiveJointStates);
         
         
         // Test
         for(int segment = 0; segment < sizes.length; segment++)
         {
            RegistrySendBuffer sendBuffer = new RegistrySendBuffer(1, sendRegistry.getAllVariables(), sendJointHolders);
            sendBuffer.updateBufferFromVariables(timestamp, uid, segment, offsets[segment], sizes[segment]);
            payload.getData().clear();
            publisherType.serialize(sendBuffer, payload);
            
            RegistryReceiveBuffer receiveBuffer = new RegistryReceiveBuffer(sendBuffer.getTimestamp());
            subscriberType.deserialize(payload, receiveBuffer);
            registryDecompressor.decompressSegment(receiveBuffer, 0);

         }
         
         List<YoVariable<?>> sendVariables = sendRegistry.getAllVariables();
         List<YoVariable<?>> receiveVariables = receiveRegistry.getAllVariables();
         
         assertEquals(sendVariables.size(), receiveVariables.size());
         for(int t = 0; t < sendVariables.size(); t++)
         {
            assertEquals(sendVariables.get(t).getValueAsLongBits(), receiveVariables.get(t).getValueAsLongBits());
         }
         
      }
   }
}
