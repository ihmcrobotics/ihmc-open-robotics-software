package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public abstract class QuadrupedUnconstrainedFootState extends QuadrupedFootState
{
   protected final QuadrupedSolePositionController solePositionController;
   protected final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;

   protected final FrameVector3D initialSoleForces = new FrameVector3D();

   protected final FrameVector3D desiredLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public QuadrupedUnconstrainedFootState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox,
                                          QuadrupedSolePositionController solePositionController)
   {
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);
   }

   public void doControl()
   {
   }
}
