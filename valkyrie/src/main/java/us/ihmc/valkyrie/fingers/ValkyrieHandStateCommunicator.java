package us.ihmc.valkyrie.fingers;

import java.io.IOException;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.valkyrieRosControl.ValkyriePriorityParameters;

public class ValkyrieHandStateCommunicator
{
   private final PeriodicRealtimeThreadScheduler scheduler = new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.HAND_COMMUNICATOR_PRIORITY);
   private final SideDependentList<ConcurrentRingBuffer<HandJointAnglePacket>> packetBuffers = new SideDependentList<>();
   private final SideDependentList<HandJointAnglePacket> packetsForPublish = new SideDependentList<>();

   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDoFJoint>> handJoints = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private GlobalDataProducer dataProducer;

   public ValkyrieHandStateCommunicator(FloatingInverseDynamicsJoint rootJoint, ValkyrieHandModel handModel)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor());

      for (RobotSide robotside : RobotSide.values)
      {
         HandJointAnglePacketBuilder builder = new HandJointAnglePacketBuilder();
         packetsForPublish.put(robotside, builder.newInstance());
         packetBuffers.put(robotside, new ConcurrentRingBuffer<>(builder, 4));

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            InverseDynamicsJoint[] foundJoints = ScrewTools.findJointsWithNames(allJoints, jointEnum.getJointName(robotside));
            if (foundJoints.length != 1)
               throw new RuntimeException("Expected to find one joint with the name: " + jointEnum.getJointName(robotside) + ", got: "
                     + Arrays.toString(foundJoints));
            handJoints.get(robotside).put(jointEnum, (OneDoFJoint) foundJoints[0]);
         }
      }
   }

   public void start()
   {
      scheduler.schedule(this::publish, 10, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
      scheduler.shutdown();
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConcurrentRingBuffer<HandJointAnglePacket> buffer = packetBuffers.get(robotSide);
         HandJointAnglePacket packet = buffer.next();
         if (packet != null)
         {
            for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
            {
               double q = handJoints.get(robotSide).get(jointEnum).getQ();
               packet.jointAngles[jointEnum.getIndex(robotSide)] = q;
            }
            buffer.commit();
         }
      }
   }

   private void publish()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConcurrentRingBuffer<HandJointAnglePacket> buffer = packetBuffers.get(robotSide);

         if (buffer.poll())
         {
            boolean hasPacketToPublish = false;
            HandJointAnglePacket packet;

            while ((packet = buffer.read()) != null)
            {
               hasPacketToPublish = true;
               packetsForPublish.get(robotSide).set(packet);
            }
            buffer.flush();

            if (hasPacketToPublish)
            {
               try
               {
                  dataProducer.skipQueueAndSend(packetsForPublish.get(robotSide));
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
         }
      }
   }

   private class HandJointAnglePacketBuilder implements Builder<HandJointAnglePacket>
   {
      @Override
      public HandJointAnglePacket newInstance()
      {
         double[] handJoints = new double[ValkyrieHandJointName.values.length];
         return new HandJointAnglePacket(null, false, false, handJoints);
      }
   }
}
