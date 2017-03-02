package us.ihmc.exampleSimulations.skippy;

import us.ihmc.euclid.tuple3D.Point3D;
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
      Point3D footLocation = skippyRobot.computeFootLocation();
      Point3D com = new Point3D();
      skippyRobot.computeCenterOfMass(com);

      boolean skippyFalling = com.getZ() < footLocation.getZ() + 0.1;
      if (skippyFalling)
         controllerFailureListener.controllerFailed(null);
   }
}
