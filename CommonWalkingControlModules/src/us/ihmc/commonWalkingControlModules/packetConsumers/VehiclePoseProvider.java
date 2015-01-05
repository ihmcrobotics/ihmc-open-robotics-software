package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.driving.VehiclePosePacket;

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
