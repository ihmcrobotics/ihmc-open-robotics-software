package us.ihmc.aware.controller.force;

import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class QuadrupedForceControllerContext
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final DivergentComponentOfMotionController dcmPositionController;

   public QuadrupedForceControllerContext(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters parameters, YoVariableRegistry registry)
   {
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();
      double gravity = 9.81;

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), parameters.getJointMap(), parameters.getPhysicalProperties());
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, parameters.getJointMap(), registry);
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment.getFullRobotModel(), referenceFrames, parameters.getJointMap(), parameters.getQuadrupedJointLimits(), runtimeEnvironment.getControlDT(), registry);
      dcmPositionController = new DivergentComponentOfMotionController("dcmPosition", referenceFrames.getCenterOfMassZUpFrame(), runtimeEnvironment.getControlDT(), mass, gravity, 1.0, registry);

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
}
