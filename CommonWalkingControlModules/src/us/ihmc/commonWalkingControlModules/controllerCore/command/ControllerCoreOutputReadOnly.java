package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface ControllerCoreOutputReadOnly
{

   void getDesiredCenterOfPressure(FramePoint2d copToPack, RigidBody rigidBody);

   void getLinearMomentumRate(FrameVector linearMomentumRateToPack);

}