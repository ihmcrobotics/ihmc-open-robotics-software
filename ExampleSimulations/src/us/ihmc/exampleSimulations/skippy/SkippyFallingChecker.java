package us.ihmc.exampleSimulations.skippy;

import javax.vecmath.Point3d;

import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class SkippyFallingChecker extends SimpleRobotController
{
   private final ControllerFailureListener controllerFailureListener;
   private final SkippyRobot skippyRobot;

   public SkippyFallingChecker(ControllerFailureListener controllerFailureListener, SkippyRobot skippyRobot)
   {
      this.controllerFailureListener = controllerFailureListener;
      this.skippyRobot = skippyRobot;
   }

   @Override
   public void doControl()
   {
      Point3d footLocation = skippyRobot.computeFootLocation();
      Point3d com = new Point3d();
      skippyRobot.computeCenterOfMass(com);

      boolean skippyFalling = com.getZ() < footLocation.getZ() + 0.1;
      if (skippyFalling)
         controllerFailureListener.controllerFailed(null);
   }
}
