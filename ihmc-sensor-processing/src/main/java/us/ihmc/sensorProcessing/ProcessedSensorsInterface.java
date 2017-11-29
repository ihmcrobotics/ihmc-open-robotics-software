package us.ihmc.sensorProcessing;

import java.util.HashMap;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.sensors.FingerForceSensors;

public interface ProcessedSensorsInterface extends FingerForceSensors
{
   public abstract double getTime();
   public abstract YoDouble getYoTime();
   
   public abstract double getKneeAngle(RobotSide robotSide);

   public abstract FramePoint3D getCenterOfMassPositionInFrame(ReferenceFrame referenceFrame);

   public abstract FrameVector3D getGravityInWorldFrame();

   public abstract Twist getTwistOfPelvisWithRespectToWorld();
   public abstract SpatialAccelerationVector getAccelerationOfPelvisWithRespectToWorld();

   public abstract FrameQuaternion getPelvisOrientationInFrame(ReferenceFrame referenceFrame);

   public abstract FramePoint3D getCenterOfMassGroundProjectionInFrame(ReferenceFrame referenceFrame);

   public abstract double getLegJointPosition(RobotSide robotSide, LegJointName legJointName);
   public abstract double getLegJointVelocity(RobotSide robotSide, LegJointName legJointName);

   public abstract double getArmJointPosition(RobotSide robotSide, ArmJointName legJointName);
   public abstract double getArmJointVelocity(RobotSide robotSide, ArmJointName legJointName);


   public abstract double getTotalMass();
   
   public abstract String getLegJointPositionName(RobotSide robotSide, LegJointName jointName);

   public abstract String getLegJointVelocityName(RobotSide robotSide, LegJointName jointName);

   public abstract FrameQuaternion getChestOrientationInFrame(ReferenceFrame desiredHeadingFrame);
   public abstract FrameVector3D getChestAngularVelocityInChestFrame();

   public abstract double getSpineJointPosition(SpineJointName spineJointName);
   public abstract double getSpineJointVelocity(SpineJointName spineJointName);

   public abstract double getNeckJointPosition(NeckJointName neckJointName);
   public abstract double getNeckJointVelocity(NeckJointName neckJointName);
   public abstract FullHumanoidRobotModel getFullRobotModel();
   
   public abstract FrameVector3D getBodyVelocity();
   public abstract FrameVector3D getCenterOfMassVelocityInFrame(ReferenceFrame referenceFrame);
   
   public abstract FramePoint3D getCentroidalMomentPivotInFrame(ReferenceFrame referenceFrame);
   public abstract FrameVector3D getAngularMomentumInFrame(ReferenceFrame midFeetZUp);
   
   public abstract HashMap<FramePoint2D, Boolean> getContactMap(RobotSide robotSide);
}
