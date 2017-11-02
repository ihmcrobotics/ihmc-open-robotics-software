package us.ihmc.valkyrieRosControl;

import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;

public class ValkyrieFingerStateCommunicator
{
   private final PeriodicRealtimeThreadScheduler scheduler = new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.HAND_COMMUNICATOR_PRIORITY);
   private final ConcurrentRingBuffer<HandJointAnglePacket> packetBuffer = new ConcurrentRingBuffer<>(new HandJointAnglePacketBuilder(), 4);

   public ValkyrieFingerStateCommunicator(FloatingInverseDynamicsJoint rootJoint, ValkyrieHandModel handModel)
   {
      
   }

   private class HandJointAnglePacketBuilder implements Builder<HandJointAnglePacket>
   {
      public HandJointAnglePacketBuilder()
      {
      }

      @Override
      public HandJointAnglePacket newInstance()
      {
         return new HandJointAnglePacket(null, false, false, new double[ValkyrieHandJointName.values.length]);
      }
   }
}
