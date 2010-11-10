package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

public interface ProcessedSensorsInterface
{
   public abstract double getTime();

   public abstract double getKneeAngle(RobotSide robotSide);

   public abstract FrameVector getCenterOfMassVelocityInFrame(ReferenceFrame referenceFrame);

   public abstract FramePoint getCenterOfMassPositionInFrame(ReferenceFrame referenceFrame);

   public abstract FrameVector getGravityInWorldFrame();

   public abstract Twist computeTwistOfPelvisWithRespectToWorld();

   public abstract Orientation getPelvisOrientationInFrame(ReferenceFrame referenceFrame);

   public abstract FramePoint getCenterOfMassGroundProjectionInFrame(ReferenceFrame referenceFrame);

   public abstract double getLegJointVelocity(RobotSide robotSide, LegJointName legJointName);

   public abstract double getLegJointPosition(RobotSide robotSide, LegJointName legJointName);

   public abstract double getTotalMass();
   
   public abstract String getLegJointPositionName(RobotSide robotSide, LegJointName jointName);

   public abstract String getLegJointVelocityName(RobotSide robotSide, LegJointName jointName);

}
