package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;
import java.util.EnumMap;

public class DRCVehicleModelObjects
{
   private static EnumMap<VehcileObject, Transform3D> objectTransforms;

   static
   {
      objectTransforms = new EnumMap<VehcileObject, Transform3D>(VehcileObject.class);


      {
         Transform3D transform3D = new Transform3D();
         double roll = 0.0;
         double pitch = 0.0;
         double yaw = 0.0;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.0, 0.0, 0.0);
         transform3D.set(translation);

         objectTransforms.put(VehcileObject.ORIGIN, transform3D);
      }

      {
         Transform3D transform3D = new Transform3D();
         double roll = 0.0;
         double pitch = -1.0;
         double yaw = 0.0;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.580000, 0.140000, 0.510000);
         transform3D.set(translation);

         objectTransforms.put(VehcileObject.GAS_PEDAL, transform3D);
      }

      {
         Transform3D transform3D = new Transform3D();
         double roll = 0.0;
         double pitch = -1.0;
         double yaw = 0.0;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.590000, 0.270000, 0.530000);
         transform3D.set(translation);

         objectTransforms.put(VehcileObject.BRAKE_PEDAL, transform3D);
      }

      {
         Transform3D transform3D = new Transform3D();
         double roll = 0.0;
         double pitch = -0.8;
         double yaw = 0.0;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.370000, 0.300000, 1.200000);
         transform3D.set(translation);

         objectTransforms.put(VehcileObject.STEERING_WHEEL, transform3D);
      }

      {
         Transform3D transform3D = new Transform3D();
         double roll = 3.141590;
         double pitch = -0.058407;
         double yaw = -3.141590;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.500000, 0.000000, 1.050000);
         transform3D.set(translation);

         objectTransforms.put(VehcileObject.HAND_BRAKE, transform3D);
      }
   }

   public static Transform3D getTransform(VehcileObject vehcileObject)
   {
      return new Transform3D(objectTransforms.get(vehcileObject));
   }


   public enum VehcileObject
   {
      ORIGIN, GAS_PEDAL, BRAKE_PEDAL, STEERING_WHEEL, HAND_BRAKE,
   }
}
