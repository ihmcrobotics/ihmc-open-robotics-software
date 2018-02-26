package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieHandStateCommunicator implements RobotController
{
   private final PeriodicRealtimeThreadScheduler scheduler;
   private final SideDependentList<ConcurrentRingBuffer<HandJointAnglePacket>> packetBuffers = new SideDependentList<>();
   private final SideDependentList<HandJointAnglePacket> packetsForPublish = new SideDependentList<>();

   private final SideDependentList<EnumMap<ValkyrieHandJointName, OneDoFJoint>> handJoints = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private final PacketCommunicator packetCommunicator;

   private boolean isRunning = false;

   public ValkyrieHandStateCommunicator(FullHumanoidRobotModel fullRobotModel, ValkyrieHandModel handModel, PacketCommunicator packetCommunicator, PeriodicRealtimeThreadScheduler scheduler)
   {
      this.packetCommunicator = packetCommunicator;
      this.scheduler = scheduler;

      for (RobotSide robotside : RobotSide.values)
      {
         HandJointAnglePacketBuilder builder = new HandJointAnglePacketBuilder();
         packetsForPublish.put(robotside, builder.newInstance());
         packetBuffers.put(robotside, new ConcurrentRingBuffer<>(builder, 4));

         for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
         {
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointEnum.getJointName(robotside));
            handJoints.get(robotside).put(jointEnum, joint);
         }
      }
   }

   public void start()
   {
      if (isRunning)
         return;

      scheduler.schedule(this::publish, 10, TimeUnit.MILLISECONDS);
      isRunning = true;
   }

   public void stop()
   {
      scheduler.shutdown();
      isRunning = false;
   }

   @Override
   public void initialize()
   {
      start();
   }

   @Override
   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConcurrentRingBuffer<HandJointAnglePacket> buffer = packetBuffers.get(robotSide);
         HandJointAnglePacket packet = buffer.next();
         if (packet != null)
         {
            packet.robotSide = robotSide;

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
               packetCommunicator.send(packetsForPublish.get(robotSide));
            }
         }
      }
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   private class HandJointAnglePacketBuilder implements Builder<HandJointAnglePacket>
   {
      @Override
      public HandJointAnglePacket newInstance()
      {
         double[] handJoints = new double[ValkyrieHandJointName.values.length];
         return HumanoidMessageTools.createHandJointAnglePacket(null, false, false, handJoints);
      }
   }
}
