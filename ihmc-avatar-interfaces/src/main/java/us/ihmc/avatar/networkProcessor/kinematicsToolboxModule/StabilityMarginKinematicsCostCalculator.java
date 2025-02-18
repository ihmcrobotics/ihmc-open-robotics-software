package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedStabilityGradientCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StabilityMarginKinematicsCostCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private static final double NULL_POSTURAL_SENSITIVITY = -1.0;

   private final WholeBodyContactState wholeBodyContactState;
   private final StabilityMarginRegionCalculator multiContactRegionCalculator;
   private final SensitivityBasedStabilityGradientCalculator stabilityGradientCalculator;
   private final FullHumanoidRobotModel fullRobotModel;

   private final BooleanProvider isUpperBodyLoadBearing;
   private final DoubleProvider minStabilityMargin;

   private final YoBoolean isEnabled = new YoBoolean("isStabilityObjectiveEnabled", registry);
   private final YoDouble desiredStabilityMarginVelocity = new YoDouble("desiredStabilityMarginVelocity", registry);
   private final YoDouble stabilityMarginWeight = new YoDouble("stabilityMarginWeight", registry);
   private final YoDouble stabilityMarginThreshold = new YoDouble("stabilityMarginThreshold", registry);
   private final YoDouble stabilityMarginHysteresis = new YoDouble("stabilityMarginHysteresis", registry);

   private final FramePose3D pelvisControlFramePose = new FramePose3D();
   private final FramePose3D pelvisPose = new FramePose3D();
   private final SpatialVector desiredPelvisSpatialVelocity = new SpatialVector();

   public StabilityMarginKinematicsCostCalculator(WholeBodyContactState wholeBodyContactState,
                                                  StabilityMarginRegionCalculator multiContactRegionCalculator,
                                                  FullHumanoidRobotModel fullRobotModel,
                                                  BooleanProvider isUpperBodyLoadBearing,
                                                  DoubleProvider minStabilityMargin,
                                                  YoRegistry parentRegistry)
   {
      this.wholeBodyContactState = wholeBodyContactState;
      this.multiContactRegionCalculator = multiContactRegionCalculator;
      this.fullRobotModel = fullRobotModel;

      this.isUpperBodyLoadBearing = isUpperBodyLoadBearing;
      this.minStabilityMargin = minStabilityMargin;

      desiredStabilityMarginVelocity.set(0.15);
      stabilityMarginThreshold.set(0.12);
      stabilityMarginHysteresis.set(0.03);
      stabilityMarginWeight.set(0.5);

      MovingReferenceFrame afterRootJointFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      pelvisControlFramePose.setToZero(afterRootJointFrame);
      pelvisControlFramePose.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());

      this.stabilityGradientCalculator = new SensitivityBasedStabilityGradientCalculator(fullRobotModel,
                                                                                         wholeBodyContactState,
                                                                                         multiContactRegionCalculator,
                                                                                         registry);
      parentRegistry.addChild(registry);
   }

   public void setEnabled(boolean enable)
   {
      isEnabled.set(enable);
   }

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   /**
    * Computes and packs the feedback objective. Returns the postural sensitivity
    */
   public double addPostureFeedbackCommands(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!isUpperBodyLoadBearing.getValue() || !multiContactRegionCalculator.hasSolvedWholeRegion())
         return NULL_POSTURAL_SENSITIVITY;

      double stabilityMargin = multiContactRegionCalculator.getStabilityMargin();
      double deltaStabilityMargin = stabilityMargin - minStabilityMargin.getValue();
      double alpha = EuclidCoreTools.clamp(deltaStabilityMargin / minStabilityMargin.getValue(), 0.0, 1.0);
      double weight = alpha * stabilityMarginWeight.getValue();

      stabilityGradientCalculator.update();
      double posturalSensitivity = stabilityGradientCalculator.getPostureSensitivity();

      if (!isEnabled.getValue() || posturalSensitivity < 1.0e-3)
         return posturalSensitivity;

      DMatrixRMaj stabilityMarginGradient = stabilityGradientCalculator.getStabilityMarginGradient();
      double gradientScalar = desiredStabilityMarginVelocity.getValue() / EuclidCoreTools.square(posturalSensitivity);

      // Feed-forward joint velocities
      OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();
      for (int joint_idx = 0; joint_idx < wholeBodyContactState.getNumberOfJoints(); joint_idx++)
      {
         OneDoFJointBasics joint = oneDoFJoints[joint_idx];
         OneDoFJointFeedbackControlCommand jointFeedbackCommand = bufferToPack.addOneDoFJointFeedbackControlCommand();
         jointFeedbackCommand.setJoint(joint);
         jointFeedbackCommand.setWeightForSolver(weight);

         int jointIndex = Twist.SIZE + joint_idx;
         double qd_ff = gradientScalar * stabilityMarginGradient.get(jointIndex);
         jointFeedbackCommand.setInverseKinematics(joint.getQ(), qd_ff);

         // Clear gains - this is all feed-forward velocity
         jointFeedbackCommand.getGains().setKp(0.0);
         jointFeedbackCommand.getGains().setKd(0.0);
      }

      // Feed-forward pelvis velocity
      pelvisPose.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPelvisSpatialVelocity.set(stabilityMarginGradient);
      desiredPelvisSpatialVelocity.scale(gradientScalar);

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = bufferToPack.addSpatialFeedbackControlCommand();
      spatialFeedbackControlCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getPelvis());
      spatialFeedbackControlCommand.setInverseKinematics(pelvisPose, desiredPelvisSpatialVelocity);
      spatialFeedbackControlCommand.setWeightForSolver(weight);
      spatialFeedbackControlCommand.getControlFramePose().set(pelvisControlFramePose);

      // Clear gains - this is all feed-forward velocity
      spatialFeedbackControlCommand.getGains().getPositionGains().setProportionalGains(0.0);
      spatialFeedbackControlCommand.getGains().getPositionGains().setDerivativeGains(0.0);
      spatialFeedbackControlCommand.getGains().getOrientationGains().setProportionalGains(0.0);
      spatialFeedbackControlCommand.getGains().getOrientationGains().setDerivativeGains(0.0);

      return posturalSensitivity;
   }
}
