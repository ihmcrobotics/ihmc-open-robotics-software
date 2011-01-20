package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public interface DesiredHeadingControlModule
{
   public abstract void updateDesiredHeadingFrame();

   public abstract ReferenceFrame getDesiredHeadingFrame();

   public abstract FrameVector getDisplacementWithRespectToFoot(RobotSide robotSide, FramePoint position);

   public abstract FrameVector getFinalHeadingTarget();

   public abstract void setFinalDesiredHeading(double desiredHeading);

   public abstract DoubleYoVariable getDesiredHeading();
}
