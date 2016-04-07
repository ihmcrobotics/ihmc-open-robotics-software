package us.ihmc.aware.controller.force;

import us.ihmc.aware.controller.toolbox.*;
import us.ihmc.aware.controller.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
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

   public QuadrupedForceControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters parameters, YoVariableRegistry registry)
   {
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();
      double gravity = 9.81;

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), parameters.getJointMap(), parameters.getPhysicalProperties());
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, parameters.getJointMap(), registry);
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment.getFullRobotModel(), referenceFrames, parameters.getJointMap(), parameters.getQuadrupedJointLimits(), registry);
      bodyOrientationController = new QuadrupedBodyOrientationController(referenceFrames.getBodyFrame(), runtimeEnvironment.getControlDT(), registry);
      dcmPositionController = new DivergentComponentOfMotionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), mass, gravity, 1.0, registry);
      solePositionController = new QuadrupedSolePositionController(referenceFrames.getFootReferenceFrames(), runtimeEnvironment.getControlDT(), registry);
      comPositionController = new QuadrupedComPositionController(referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), registry);
      timedStepController = new QuadrupedTimedStepController(runtimeEnvironment.getRobotTimestamp(), registry);

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

   public QuadrupedComPositionController getComPositionController()
   {
      return comPositionController;
   }

   public QuadrupedTimedStepController getTimedStepController()
   {
      return timedStepController;
   }
}
