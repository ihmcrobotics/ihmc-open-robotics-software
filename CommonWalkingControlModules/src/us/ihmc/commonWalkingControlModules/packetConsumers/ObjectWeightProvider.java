package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.robotics.robotSide.RobotSide;

public class ObjectWeightProvider implements PacketConsumer<ObjectWeightPacket>
{
   private final AtomicReference<ObjectWeightPacket> objectWeightPackage = new AtomicReference<ObjectWeightPacket>();
   
   private RobotSide robotSide;
   private double weight;
   
   public boolean isNewInformationAvailable()
   {
      ObjectWeightPacket packet = objectWeightPackage.getAndSet(null);
      
      if (packet != null)
      {
         robotSide = packet.getRobotSide();
         weight = packet.getWeight();
         return true;
      }
      
      return false;
   }
   
   public RobotSide getRobotSide()
   {
      return robotSide;
   }
   
   public double getWeight()
   {
      return weight;
   }

   @Override
   public void receivedPacket(ObjectWeightPacket packet)
   {
      objectWeightPackage.set(packet);
   }
}
