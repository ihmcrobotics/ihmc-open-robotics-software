package us.ihmc.quadrupedRobotics.controller.force;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

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

   public QuadrupedForceControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                          YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      this.runtimeEnvironment = runtimeEnvironment;

      footControlModuleParameters = new QuadrupedFootControlModuleParameters();
      runtimeEnvironment.getParentRegistry().addChild(footControlModuleParameters.getYoVariableRegistry());

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, registry, runtimeEnvironment.getGraphicsListRegistry());
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment, referenceFrames, registry, runtimeEnvironment.getGraphicsListRegistry());
      linearInvertedPendulumModel = new LinearInvertedPendulumModel(referenceFrames.getCenterOfMassZUpFrame(), mass, gravity, 1.0, registry);
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassZUpFrame(), linearInvertedPendulumModel, registry, yoGraphicsListRegistry);
      groundPlaneEstimator = new GroundPlaneEstimator(registry, runtimeEnvironment.getGraphicsListRegistry());
      fallDetector = new QuadrupedFallDetector(taskSpaceEstimator, dcmPositionEstimator, registry);
   }

   public void update()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);

      dcmPositionEstimator.compute(taskSpaceEstimates.getComVelocity());
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

   public QuadrupedTaskSpaceController getTaskSpaceController()
   {
      return taskSpaceController;
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return linearInvertedPendulumModel;
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

   public ReferenceFrame getSoleReferenceFrame(RobotQuadrant robotQuadrant)
   {
      return referenceFrames.getSoleFrame(robotQuadrant);
   }

   public void getDCMPositionEstimate(FramePoint3D dcmPositionEstimateToPack)
   {
      dcmPositionEstimator.getDCMPositionEstimate(dcmPositionEstimateToPack);
   }
}
