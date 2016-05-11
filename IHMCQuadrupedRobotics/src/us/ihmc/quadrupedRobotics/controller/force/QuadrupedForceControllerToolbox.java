package us.ihmc.quadrupedRobotics.controller.force;

import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class QuadrupedForceControllerToolbox
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedTimedStepController timedStepController;

   public QuadrupedForceControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties, YoVariableRegistry registry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, registry);
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment.getFullRobotModel(), referenceFrames, runtimeEnvironment.getControlDT(), registry);
      comPositionController = new QuadrupedComPositionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), registry);
      dcmPositionController = new DivergentComponentOfMotionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), mass, gravity, 1.0, registry);
      bodyOrientationController = new QuadrupedBodyOrientationController(referenceFrames.getBodyFrame(), runtimeEnvironment.getControlDT(), registry);
      solePositionController = new QuadrupedSolePositionController(referenceFrames.getFootReferenceFrames(), runtimeEnvironment.getControlDT(), registry);
      timedStepController = new QuadrupedTimedStepController(solePositionController, runtimeEnvironment.getRobotTimestamp(), registry);

      // register controller graphics
      taskSpaceController.registerGraphics(runtimeEnvironment.getGraphicsListRegistry());
      dcmPositionController.registerGraphics(runtimeEnvironment.getGraphicsListRegistry());
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public QuadrupedTaskSpaceEstimator getTaskSpaceEstimator()
   {
      return taskSpaceEstimator;
   }

   public QuadrupedTaskSpaceController getTaskSpaceController()
   {
      return taskSpaceController;
   }

   public QuadrupedComPositionController getComPositionController()
   {
      return comPositionController;
   }

   public DivergentComponentOfMotionController getDcmPositionController()
   {
      return dcmPositionController;
   }

   public QuadrupedBodyOrientationController getBodyOrientationController()
   {
      return bodyOrientationController;
   }

   public QuadrupedSolePositionController getSolePositionController()
   {
      return solePositionController;
   }

   public QuadrupedTimedStepController getTimedStepController()
   {
      return timedStepController;
   }
}
