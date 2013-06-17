package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.VehiclePosePacket;
import us.ihmc.utilities.net.ObjectConsumer;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

public class PelvisPoseWithRespectToVehicleProvider implements ObjectConsumer<VehiclePosePacket>
{
   private final Transform3D transformFromPelvisToVehicle = new Transform3D();
   private boolean hasNewPose;

   public synchronized void consumeObject(VehiclePosePacket object)
   {
      transformFromPelvisToVehicle.set(object.getOrientation());
      transformFromPelvisToVehicle.setTranslation(new Vector3d(object.getPosition()));

      hasNewPose = true;
   }

   public synchronized boolean checkForNewPose()
   {
      return hasNewPose;
   }

   public synchronized Transform3D getTransformFromPelvisToVehicle()
   {
      hasNewPose = false;
      return transformFromPelvisToVehicle;
   }
}
