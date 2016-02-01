package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class VehiclePoseProvider implements PacketConsumer<VehiclePosePacket>
{
   private final AtomicReference<RigidBodyTransform> transformFromVehicleToWorld = new AtomicReference<RigidBodyTransform>();

   public void receivedPacket(VehiclePosePacket object)
   {
      transformFromVehicleToWorld.set(new RigidBodyTransform(object.getOrientation(), new Vector3d(object.getPosition())));
   }

   public synchronized boolean checkForNewPose()
   {
      return transformFromVehicleToWorld.get() != null;
   }

   public RigidBodyTransform getTransformFromVehicleToWorld()
   {
      return transformFromVehicleToWorld.getAndSet(null);
   }
}
