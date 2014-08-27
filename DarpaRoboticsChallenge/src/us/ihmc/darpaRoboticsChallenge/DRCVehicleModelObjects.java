package us.ihmc.darpaRoboticsChallenge;

import java.util.EnumMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.VehicleModelObjects;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.VehicleObject;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class DRCVehicleModelObjects implements VehicleModelObjects
{
   private EnumMap<VehicleObject, Transform3D> objectTransforms;

   public DRCVehicleModelObjects()
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
         //0.630000 0.100000 0.580000 0.000000 0.000000 0.000000
         //-0.038500 0.000000 -0.086000 -3.141592 1.125593 -3.141592

         //parent transform
         Transform3D transform3DfromWorldToParent;
         {
         double roll = 0.0;
         double pitch = 0.0;
         double yaw = 0.0;
         Vector3d translation = new Vector3d(0.630000, 0.100000, 0.580000);
         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromWorldToParent = new Transform3D(matrix3d, translation, 1.0);
         }

         //transform to parent
         Transform3D transform3DfromParentToChild;
         {
            double roll = -3.141592;
            double pitch = 1.125593;
            double yaw = -3.141592;
            Vector3d translation = new Vector3d(-0.038500, 0.000000, -0.086000);
            Matrix3d matrix3d = new Matrix3d();
            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromParentToChild = new Transform3D(matrix3d, translation, 1.0);
         }

         Transform3D transform3D = new Transform3D();
         transform3D.mul(transform3DfromWorldToParent, transform3DfromParentToChild);

         //Rotate to have the Z axis point out
         Transform3D finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, Math.PI, 0.0));
         transform3D.mul(finalAdjustment);

         finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
         transform3D.mul(finalAdjustment);

         objectTransforms.put(VehicleObject.GAS_PEDAL, transform3D);
      }

      {
         //0.640000 0.270000 0.580000 0.000000 0.000000 0.000000
         //-0.040000 0.000000 -0.086000 -3.141592 1.142593 -3.141592
         //parent transform
         Transform3D transform3DfromWorldToParent;
         {
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            Vector3d translation = new Vector3d(0.640000, 0.270000, 0.580000);
            Matrix3d matrix3d = new Matrix3d();
            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromWorldToParent = new Transform3D(matrix3d, translation, 1.0);
         }

         //transform to parent
         Transform3D transform3DfromParentToChild;
         {
            double roll = -3.141592;
            double pitch = 1.142593;
            double yaw = -3.141592;
            Vector3d translation = new Vector3d(-0.040000, 0.000000, -0.086000);
            Matrix3d matrix3d = new Matrix3d();
            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromParentToChild = new Transform3D(matrix3d, translation, 1.0);
         }

         Transform3D transform3D = new Transform3D();
         transform3D.mul(transform3DfromWorldToParent, transform3DfromParentToChild);

         //Rotate to have the Z axis point out
         Transform3D finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, Math.PI, 0.0));
         transform3D.mul(finalAdjustment);

         finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
         transform3D.mul(finalAdjustment);

         objectTransforms.put(VehicleObject.BRAKE_PEDAL, transform3D);
      }

      {
         //0.340000 0.300000 1.290000 0.000000 -0.870000 0.000000
         //0.000000 0.000000 0.000000 -0.690000 0.000000 -1.570796

         Transform3D transform3DfromWorldToParent;
         {
            double roll = 0.0;
            double pitch = -0.870000;
            double yaw = 0.0;
            Vector3d translation = new Vector3d(0.340000, 0.300000, 1.290000);
            Matrix3d matrix3d = new Matrix3d();
            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromWorldToParent = new Transform3D(matrix3d, translation, 1.0);
         }
         Transform3D transform3D = new Transform3D(transform3DfromWorldToParent);

         Transform3D finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
         transform3D.mul(finalAdjustment);

         //transform to parent
//         Transform3D transform3DfromParentToChild;
//         {
//            double roll = -0.690000;
//            double pitch = 0.0;
//            double yaw = -1.570796;
//            Vector3d translation = new Vector3d(0.0, 0.0, 0.0);
//            Matrix3d matrix3d = new Matrix3d();
//            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
//            transform3DfromParentToChild = new Transform3D(matrix3d, translation, 1.0);
//         }
//
//         Transform3D transform3D = new Transform3D();
//         transform3D.mul(transform3DfromWorldToParent, transform3DfromParentToChild);
//
//         //Rotate to have the Z axis point
//         double xRotation = Math.toRadians(29.0);
//         Transform3D finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(xRotation, 0.0, 0.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, Math.PI/2.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.set(new Vector3d(0.03, 0.0, 0.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.setTranslation(new Vector3d(0.0, -0.0225, 0.0)); // to line up the center better
//         transform3D.mul(finalAdjustment);




         objectTransforms.put(VehicleObject.STEERING_WHEEL, transform3D);
      }

      {
         //0.530000 0.070000 1.050000 0.000000 0.000000 0.000000
         //0.000000 0.000000 0.050000 -0.200000 -0.000000 -1.570796

         Transform3D transform3DfromWorldToParent;
         {
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            Vector3d translation = new Vector3d(0.530000, 0.070000, 1.050000);
            Matrix3d matrix3d = new Matrix3d();
            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
            transform3DfromWorldToParent = new Transform3D(matrix3d, translation, 1.0);
         }

//         //transform to parent
//         Transform3D transform3DfromParentToChild;
//         {
//            double roll = -0.200000;
//            double pitch = 0.0;
//            double yaw = -1.570796;
//            Vector3d translation = new Vector3d(0.000000, 0.000000, 0.050000);
//            Matrix3d matrix3d = new Matrix3d();
//            RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);
//            transform3DfromParentToChild = new Transform3D(matrix3d, translation, 1.0);
//         }

//         Transform3D transform3D = new Transform3D();
//         transform3D.mul(transform3DfromWorldToParent, transform3DfromParentToChild);
         Transform3D transform3D = new Transform3D(transform3DfromWorldToParent);

//         //Rotate to have the Z axis point out
//         Transform3D finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(Math.PI/2.0, 0.0, 0.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, Math.PI/2.0));
//         transform3D.mul(finalAdjustment);
//
//         finalAdjustment = new Transform3D();
//         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
//         transform3D.mul(finalAdjustment);

         objectTransforms.put(VehicleObject.HAND_BRAKE, transform3D);
      }

      {
         //0.560000 -0.020000 1.080000 0.000000 0.250000 0.000000
         double roll = 0.000000;
         double pitch = 0.250000;
         double yaw =  0.000000;

         Vector3d translation = new Vector3d(0.560000, -0.020000, 1.080000);
         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);

         //Rotate to have the Z axis point out
         Transform3D finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, -Math.PI/2.0, 0.0));
         transform3D.mul(finalAdjustment);

         finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
         transform3D.mul(finalAdjustment);

         Transform3D adjustmentForSwitch = new Transform3D();
         adjustmentForSwitch.setTranslation(new Vector3d(0.0, 0.025, 0.0));
         transform3D.mul(adjustmentForSwitch);

         objectTransforms.put(VehicleObject.FNR_SWITCH_F, transform3D);
      }

      {
         //0.560000 -0.020000 1.080000 0.000000 0.250000 0.000000
         double roll = 0.000000;
         double pitch = 0.250000;
         double yaw =  0.000000;

         Vector3d translation = new Vector3d(0.560000, -0.020000, 1.080000);
         Matrix3d matrix3d = new Matrix3d();
         RotationFunctions.setYawPitchRoll(matrix3d, yaw, pitch, roll);

         Transform3D transform3D = new Transform3D(matrix3d, translation, 1.0);

         //Rotate to have the Z axis point out
         Transform3D finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, -Math.PI/2.0, 0.0));
         transform3D.mul(finalAdjustment);

         finalAdjustment = new Transform3D();
         finalAdjustment.setEuler(new Vector3d(0.0, 0.0, -Math.PI/2.0));
         transform3D.mul(finalAdjustment);

         Transform3D adjustmentForSwitch = new Transform3D();
         adjustmentForSwitch.setTranslation(new Vector3d(0.0, -0.025, 0.0));
         transform3D.mul(adjustmentForSwitch);

         objectTransforms.put(VehicleObject.FNR_SWITCH_R, transform3D);
      }
   }

   public Transform3D getTransform(VehicleObject vehicleObject)
   {
      return new Transform3D(objectTransforms.get(vehicleObject));
   }

   public FramePose getFramePose(ReferenceFrame vehicleReferenceFrame, VehicleObject vehicleObject)
   {
      FramePose framePose = new FramePose(vehicleReferenceFrame);
      Transform3D transform3D = getTransform(vehicleObject);

      framePose.setPoseIncludingFrame(vehicleReferenceFrame, transform3D);

      return framePose;
   }

   public double getHandBrakeEngagedAngle()
   {
      return 0.6;
   }

   public double getHandBrakeDisengagedAngle()
   {
      return 0.0;
   }

   public Vector3d getHandBrakeAxis()
   {
      return new Vector3d(0.0, -1.0, 0.0);
   }

   public double getSteeringWheelInnerRadius()
   {
      return 0.143;
   }

   public double getSteeringWheelOuterRadius()
   {
      return 0.173;
   }

   public double getMaximumGasPedalDistance()
   {
      return -0.08;
   }

   public double getMaximumBrakePedalDistance()
   {
      return -0.08;
   }

   public double getEmergencyBrakePedalDistance()
   {
      return -0.02;
   }
}
