package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.utilities.math.geometry.Transform3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.driving.VehiclePosePacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class VehiclePoseProvider implements ObjectConsumer<VehiclePosePacket>
{
   private final AtomicReference<Transform3d> transformFromVehicleToWorld = new AtomicReference<Transform3d>();

   public void consumeObject(VehiclePosePacket object)
   {
      transformFromVehicleToWorld.set(new Transform3d(object.getOrientation(), new Vector3d(object.getPosition()), 1.0));
   }

   public synchronized boolean checkForNewPose()
   {
      return transformFromVehicleToWorld.get() != null;
   }

   public Transform3d getTransformFromVehicleToWorld()
   {
      return transformFromVehicleToWorld.getAndSet(null);
   }
}
