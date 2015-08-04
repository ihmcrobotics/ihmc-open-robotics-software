package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * @author twan
 *         Date: 5/29/13
 */
public interface VehicleModelObjects
{
   FramePose getFramePose(ReferenceFrame vehicleReferenceFrame, VehicleObject vehicleObject);

   RigidBodyTransform getTransform(VehicleObject vehicleObject);

   double getHandBrakeEngagedAngle();

   double getHandBrakeDisengagedAngle();

   Vector3d getHandBrakeAxis();

   double getSteeringWheelOuterRadius();

   double getSteeringWheelInnerRadius();
   
   double getMaximumGasPedalDistance();
   
   double getMaximumBrakePedalDistance();
   
   double getEmergencyBrakePedalDistance();
}
