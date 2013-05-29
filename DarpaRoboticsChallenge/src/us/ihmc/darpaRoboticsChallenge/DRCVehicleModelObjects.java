package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.EnumMap;

public class DRCVehicleModelObjects
{
   private static EnumMap<VehicleObject, Transform3D> objectTransforms;

   static
   {
      objectTransforms = new EnumMap<VehicleObject, Transform3D>(VehicleObject.class);


      {
         Transform3D transform3D = new Transform3D();
         double roll = 0.0;
         double pitch = 0.0;
         double yaw = 0.0;
         transform3D.setEuler(new Vector3d(roll, pitch, yaw));

         Vector3d translation = new Vector3d(0.0, 0.0, 0.0);
         transform3D.set(translation);

         objectTransforms.put(VehicleObject.ORIGIN, transform3D);
      }

      {

         double roll = 0.0;
         double pitch = -1.0;
         double yaw = 0.0;

         Vector3d translation = new Vector3d(0.580000, 0.140000, 0.510000);

         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);

         objectTransforms.put(VehicleObject.GAS_PEDAL, transform3D);
      }

      {
         double roll = 0.0;
         double pitch = -1.0;
         double yaw = 0.0;

         Vector3d translation = new Vector3d(0.590000, 0.270000, 0.530000);

         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);

         objectTransforms.put(VehicleObject.BRAKE_PEDAL, transform3D);
      }

      {
         double roll = 0.0;
         double pitch = -0.8;
         double yaw = 0.0;

         Vector3d translation = new Vector3d(0.370000, 0.300000, 1.200000);

         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);
         objectTransforms.put(VehicleObject.STEERING_WHEEL, transform3D);
      }

      {
         double roll = 3.141590;
         double pitch = -0.058407;
         double yaw = -3.141590;

         Vector3d translation = new Vector3d(0.500000, 0.000000, 1.050000);
         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);
         objectTransforms.put(VehicleObject.HAND_BRAKE, transform3D);
      }
   }

   public static Transform3D getTransform(VehicleObject vehicleObject)
   {
      return new Transform3D(objectTransforms.get(vehicleObject));
   }



   public static FramePose getFramePose(ReferenceFrame vehicleReferenceFrame, VehicleObject vehicleObject)
   {
      FramePose framePose = new FramePose(vehicleReferenceFrame);
      Transform3D transform3D = getTransform(vehicleObject);

      framePose.set(vehicleReferenceFrame, transform3D);

      return framePose;
   }


   public enum VehicleObject
   {
      ORIGIN, GAS_PEDAL, BRAKE_PEDAL, STEERING_WHEEL, HAND_BRAKE,
   }
}
