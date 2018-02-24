package us.ihmc.quadrupedRobotics.controller.force;

import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedForceControllerToolbox
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrupedFallDetector fallDetector;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedFootControlModuleParameters footControlModuleParameters;

   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();

   public QuadrupedForceControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties, YoVariableRegistry registry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      this.runtimeEnvironment = runtimeEnvironment;
      
      footControlModuleParameters = new QuadrupedFootControlModuleParameters();
      runtimeEnvironment.getParentRegistry().addChild(footControlModuleParameters.getYoVariableRegistry());

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, registry, runtimeEnvironment.getGraphicsListRegistry());
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment.getFullRobotModel(), referenceFrames, runtimeEnvironment.getControlDT(), registry, runtimeEnvironment.getGraphicsListRegistry());
      linearInvertedPendulumModel = new LinearInvertedPendulumModel(referenceFrames.getCenterOfMassZUpFrame(), mass, gravity, 1.0, registry);
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassZUpFrame(), linearInvertedPendulumModel, registry, runtimeEnvironment.getGraphicsListRegistry());
      groundPlaneEstimator = new GroundPlaneEstimator(registry, runtimeEnvironment.getGraphicsListRegistry());
      fallDetector = new QuadrupedFallDetector(taskSpaceEstimator, dcmPositionEstimator, registry);
   }

   public void update()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   public QuadrupedRuntimeEnvironment getRuntimeEnvironment()
   {
      return runtimeEnvironment;
   }

   public QuadrupedFootControlModuleParameters getFootControlModuleParameters()
   {
      return footControlModuleParameters;
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

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return linearInvertedPendulumModel;
   }

   public DivergentComponentOfMotionEstimator getDcmPositionEstimator()
   {
      return dcmPositionEstimator;
   }

   public GroundPlaneEstimator getGroundPlaneEstimator()
   {
      return groundPlaneEstimator;
   }

   public QuadrupedFallDetector getFallDetector()
   {
      return fallDetector;
   }

   public QuadrupedTaskSpaceEstimates getTaskSpaceEstimates()
   {
      return taskSpaceEstimates;
   }
}
