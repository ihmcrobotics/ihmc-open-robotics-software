package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.VehiclePosePacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class VehiclePoseProvider implements ObjectConsumer<VehiclePosePacket>
{
   private final AtomicReference<Transform3D> transformFromVehicleToWorld = new AtomicReference<Transform3D>();

   public void consumeObject(VehiclePosePacket object)
   {
      transformFromVehicleToWorld.set(new Transform3D(object.getOrientation(), new Vector3d(object.getPosition()), 1.0));
   }

   public synchronized boolean checkForNewPose()
   {
      return transformFromVehicleToWorld.get() != null;
   }

   public Transform3D getTransformFromVehicleToWorld()
   {
      return transformFromVehicleToWorld.getAndSet(null);
   }
}
