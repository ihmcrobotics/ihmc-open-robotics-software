package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
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
}
