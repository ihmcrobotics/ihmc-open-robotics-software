package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTStreamingState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedStabilityGradientCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StabilityMarginKinematicsCostCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final double MAX_ARM_ORIENTATION_OFFSET = Math.toRadians(180.0); // Math.toRadians(35.0);
   private static final double MAX_CHEST_ORIENTATION_OFFSET = Math.toRadians(180.0); // Math.toRadians(35.0);
   private static final double MAX_PELVIS_ORIENTATION_OFFSET = Math.toRadians(180.0); // Math.toRadians(35.0);

   private static final boolean OVERRIDE_MESSAGE = true;
   private static final boolean ENABLE_POSTURE_OBJECTIVE = false;
   private static final boolean ENABLE_CONTACT_OBJECTIVE = false;

   private final WholeBodyContactState wholeBodyContactState;
   private final StabilityMarginRegionCalculator multiContactRegionCalculator;
   private final SensitivityBasedStabilityGradientCalculator stabilityGradientCalculator;
   private final FullHumanoidRobotModel fullRobotModel;
   private final DoubleProvider minStabilityMargin;

   private final Vector3D chestDefaultWeight = new Vector3D();
   private final Vector3D chestMultiContactWeight = new Vector3D();
   private final YoVector3D chestWeight;
   private final SelectionMatrix3D chestSelectionMatrix = new SelectionMatrix3D();

   private final Vector3D pelvisDefaultWeight = new Vector3D();
   private final Vector3D pelvisMultiContactWeight = new Vector3D();
   private final YoVector3D pelvisWeight;
   private final SelectionMatrix3D pelvisSelectionMatrix = new SelectionMatrix3D();

   private final DefaultPID3DGains orientationGains = new DefaultPID3DGains();

   /* When enabled, will compute and update region under two conditions: preview is requested or upper body is load-bearing */
   private final YoBoolean isEnabled = new YoBoolean("isStabilityObjectiveEnabled", registry);
   /* The upper body is actively load bearing */
   private final BooleanProvider isUpperBodyLoadBearing;
   /* Compute support region and preview it at the current configuration, even if upper body is not load-bearing */
   private final YoBoolean requestSupportRegionPreview = new YoBoolean("requestSupportRegionPreview", registry);
   /* Contact normal for which support region is previewed */
   private final FrameVector3D previewContactNormal = new FrameVector3D();

   private final OrientationOffsetCalculator chestOrientationOffsetCalculator;
   private final OrientationOffsetCalculator pelvisOrientationOffsetCalculator;
   private final OrientationOffsetCalculator bracingHandOrientationOffsetCalculator;
   private final CoMHeightOffsetCalculator comHeightOffsetCalculator;

   private final JointBasics[] controlledJoints;
   private final DMatrixRMaj currentWholeBodyVelocity = new DMatrixRMaj(0);

   private final YoDouble desiredStabilityMarginVelocity = new YoDouble("desiredStabilityMarginVelocity", registry);
   private final YoDouble stabilityMarginWeight = new YoDouble("stabilityMarginWeight", registry);
   private final YoDouble stabilityMarginThreshold = new YoDouble("stabilityMarginThreshold", registry);
   private final YoDouble stabilityMarginHysteresis = new YoDouble("stabilityMarginHysteresis", registry);
   private final YoDouble alphaEnabled = new YoDouble("alphaEnabled", registry);
   private final YoDouble gain = new YoDouble("stabilityGain", registry);

   private final double updateDT;
   private final YoDouble previousStabilityMargin = new YoDouble("previousStabilityMargin", registry);
   private final YoDouble actualStabilityMarginVelocity = new YoDouble("actualStabilityMarginVelocity", registry);

   private final DMatrixRMaj scaledStabilityGradient = new DMatrixRMaj(0);
   private final OrientationCalculator chestOrientationCalculator;
   private final OrientationCalculator pelvisOrientationCalculator;

   private int bracingPointIndex;
   private final YoFrameVector3D contactPointAdjustment;
   private final YoFrameVector3D integratedContactPointAdjustment;
   private static final double MAX_CONTACT_POINT_ADJUSTMENT = 0.15;

   public StabilityMarginKinematicsCostCalculator(WholeBodyContactState wholeBodyContactState,
                                                  StabilityMarginRegionCalculator multiContactRegionCalculator,
                                                  FullHumanoidRobotModel fullRobotModel,
                                                  ReferenceFrame midFeetZUpFrame,
                                                  ReferenceFrame centerOfMassFrame,
                                                  BooleanProvider isUpperBodyLoadBearing,
                                                  DoubleProvider minStabilityMargin,
                                                  double updateDT,
                                                  YoRegistry parentRegistry)
   {
      this.wholeBodyContactState = wholeBodyContactState;
      this.multiContactRegionCalculator = multiContactRegionCalculator;
      this.fullRobotModel = fullRobotModel;
      this.controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.updateDT = updateDT;
      int dofs = MultiBodySystemTools.computeDegreesOfFreedom(controlledJoints);
      currentWholeBodyVelocity.reshape(dofs, 1);

      this.isUpperBodyLoadBearing = isUpperBodyLoadBearing;
      this.minStabilityMargin = minStabilityMargin;

      desiredStabilityMarginVelocity.set(0.15);
      stabilityMarginThreshold.set(0.16);
      stabilityMarginHysteresis.set(0.03);
      stabilityMarginWeight.set(0.5);

      this.stabilityGradientCalculator = new SensitivityBasedStabilityGradientCalculator(fullRobotModel,
                                                                                         wholeBodyContactState,
                                                                                         multiContactRegionCalculator,
                                                                                         registry);

      this.chestOrientationCalculator = new OrientationCalculator(fullRobotModel.getChest(), midFeetZUpFrame, centerOfMassFrame, fullRobotModel, isUpperBodyLoadBearing);
      this.pelvisOrientationCalculator = new OrientationCalculator(fullRobotModel.getPelvis(), midFeetZUpFrame, centerOfMassFrame, fullRobotModel, isUpperBodyLoadBearing);
      gain.set(30.0);

      double maxRate = Math.toRadians(15.0);
      chestOrientationOffsetCalculator = new OrientationOffsetCalculator("chest", updateDT, fullRobotModel.getChest().getBodyFixedFrame(), maxRate, MAX_CHEST_ORIENTATION_OFFSET, registry);
      pelvisOrientationOffsetCalculator = new OrientationOffsetCalculator("pelvis", updateDT, fullRobotModel.getPelvis().getBodyFixedFrame(), maxRate, MAX_PELVIS_ORIENTATION_OFFSET, registry);
      bracingHandOrientationOffsetCalculator = new OrientationOffsetCalculator("bracingHand", updateDT, fullRobotModel.getHandControlFrame(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE), maxRate, MAX_ARM_ORIENTATION_OFFSET, registry);
      comHeightOffsetCalculator = new CoMHeightOffsetCalculator(updateDT, 0.05, -0.07, 0.04, fullRobotModel.getTotalMass(), stabilityGradientCalculator.getCentroidalMomentumCalculator(), registry);

      contactPointAdjustment = new YoFrameVector3D("contactPointAdjustment", ReferenceFrame.getWorldFrame(), registry);
      integratedContactPointAdjustment = new YoFrameVector3D("integratedContactPointAdjustment", ReferenceFrame.getWorldFrame(), registry);

      if (OVERRIDE_MESSAGE)
         isEnabled.set(ENABLE_POSTURE_OBJECTIVE || ENABLE_CONTACT_OBJECTIVE);

      if (OVERRIDE_MESSAGE)
         requestSupportRegionPreview.set(ENABLE_CONTACT_OBJECTIVE);

      chestWeight = new YoVector3D("chestWeight", registry);
      pelvisWeight = new YoVector3D("pelvisWeight", registry);

      previewContactNormal.set(-0.342, 0.940,  0.000);

      // Detune the default KST values a bit
      chestDefaultWeight.set(0.0, 0.0, 0.5);
      pelvisDefaultWeight.set(1.0, 1.0, 1.0);

      // Custom lower weights to increase sensitivity
      chestMultiContactWeight.set(0.0, 0.0, 0.1);
      pelvisMultiContactWeight.set(0.2, 0.2, 0.2);

      orientationGains.setProportionalGains(1200.0);

      parentRegistry.addChild(registry);
   }

   public void setEnabled(boolean enable)
   {
      if (!OVERRIDE_MESSAGE)
         isEnabled.set(enable);
   }

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   public double getAlphaEnabled()
   {
      return alphaEnabled.getValue();
   }

   public double getPostureSensitivity()
   {
      return stabilityGradientCalculator.getPostureSensitivity();
   }

   public FrameVector3DReadOnly getPreviewContactNormal()
   {
      return previewContactNormal;
   }

   public void initialize()
   {
      chestOrientationCalculator.initialize();
      pelvisOrientationCalculator.initialize();

      // update chest orientation
      chestWeight.set(isUpperBodyLoadBearing.getValue() ? chestMultiContactWeight : chestDefaultWeight);
      configureSelectionMatrix(chestSelectionMatrix, chestWeight);

      // update pelvis orientation
      pelvisWeight.set(isUpperBodyLoadBearing.getValue() ? pelvisMultiContactWeight : pelvisDefaultWeight);
      configureSelectionMatrix(pelvisSelectionMatrix, pelvisWeight);

      integratedContactPointAdjustment.setToZero();
      alphaEnabled.set(0.0);

      previousStabilityMargin.setToNaN();
   }

   public void update()
   {
      chestOrientationCalculator.update();
      pelvisOrientationCalculator.update();

      if (!multiContactRegionCalculator.hasSolvedWholeRegion())
         return;

      if (requestSupportRegionPreview.getValue())
      {
         RigidBodyBasics bracingHand = fullRobotModel.getHand(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE);
         bracingPointIndex = wholeBodyContactState.indexOf(bracingHand);
         stabilityGradientCalculator.clearContactPointsToComputeSensitivity();
         stabilityGradientCalculator.addContactPointIndexToComputeSensitivity(bracingPointIndex);

         stabilityGradientCalculator.computeContactPointAdjustment();
         contactPointAdjustment.set(stabilityGradientCalculator.getOptimalContactPointAdjustment(bracingPointIndex));
         if (contactPointAdjustment.normSquared() > 1.0e-5)
         {
            contactPointAdjustment.normalize();

            double v = 0.075;
            contactPointAdjustment.scale(v);

            double dt = 1.0e-3;
            integratedContactPointAdjustment.add(dt * contactPointAdjustment.getX(), dt * contactPointAdjustment.getY(), dt * contactPointAdjustment.getZ());

            double adjustmentSquared = integratedContactPointAdjustment.normSquared();
            double maxAdjustmentSquared = EuclidCoreTools.square(MAX_CONTACT_POINT_ADJUSTMENT);

            if (adjustmentSquared > maxAdjustmentSquared)
            {
               integratedContactPointAdjustment.scale(maxAdjustmentSquared / adjustmentSquared);
            }
         }
      }
      else
      {
         integratedContactPointAdjustment.setToZero();
      }

      // always compute, for debugging
      stabilityGradientCalculator.computePostureAdjustment();

      // Save current whole-body velocity
      MultiBodySystemTools.extractJointsState(controlledJoints, JointStateType.VELOCITY, currentWholeBodyVelocity);

      // update gradient confidence
      if (!previousStabilityMargin.isNaN())
      {
         double deltaMargin = multiContactRegionCalculator.getStabilityMargin() - previousStabilityMargin.getValue();
         actualStabilityMarginVelocity.set(EuclidCoreTools.clamp(deltaMargin / updateDT, 0.6));
      }

      boolean integrateRegargetedObjectives = isEnabled.getValue() && isUpperBodyLoadBearing.getValue() && updateAlphaEnabled() > 0.0;
      if (integrateRegargetedObjectives)
      {
         scaledStabilityGradient.set(stabilityGradientCalculator.getStabilityBoundaryGradient());
         CommonOps_DDRM.scale(EuclidCoreTools.square(alphaEnabled.getValue()) * gain.getValue(), scaledStabilityGradient);

         MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, scaledStabilityGradient);
         fullRobotModel.updateFrames();

         // Update orientation offsets
         chestOrientationOffsetCalculator.integrate();
         pelvisOrientationOffsetCalculator.integrate();
         bracingHandOrientationOffsetCalculator.integrate();
         comHeightOffsetCalculator.integrate(scaledStabilityGradient);
      }
      else
      {
         alphaEnabled.set(0.0);

         double alphaLeak = 0.995;
         chestOrientationOffsetCalculator.clear(alphaLeak);
         pelvisOrientationOffsetCalculator.clear(alphaLeak);
         bracingHandOrientationOffsetCalculator.clear(alphaLeak);
         comHeightOffsetCalculator.clear(alphaLeak);
      }

      // Reset to initial velocities
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.VELOCITY, currentWholeBodyVelocity);
      previousStabilityMargin.set(multiContactRegionCalculator.getStabilityMargin());

      if (integrateRegargetedObjectives)
         fullRobotModel.updateFrames();
   }

   private static void configureSelectionMatrix(SelectionMatrix3D selectionMatrix, Tuple3DReadOnly weight)
   {
      double weightThreshold = 1.0e-3;
      selectionMatrix.selectXAxis(weight.getX() > weightThreshold);
      selectionMatrix.selectYAxis(weight.getY() > weightThreshold);
      selectionMatrix.selectZAxis(weight.getZ() > weightThreshold);
   }

   public FrameVector3DReadOnly getIntegratedContactPointAdjustment()
   {
      return integratedContactPointAdjustment;
   }

   public void disableSupportRegionPreview()
   {
      if (!OVERRIDE_MESSAGE)
         requestSupportRegionPreview.set(false);
   }

   public void enableSupportRegionPreview(Vector3D contactPointNormal)
   {
      LogTools.info("Enabling region preview");
      if (!OVERRIDE_MESSAGE)
      {
         requestSupportRegionPreview.set(true);
         previewContactNormal.set(contactPointNormal);
      }
   }

   public boolean previewSupportRegion()
   {
      return requestSupportRegionPreview.getValue();
   }

   public void retargetRigidBody(KinematicsToolboxRigidBodyCommand command)
   {
      boolean isBracingHand = command.getEndEffector().equals(fullRobotModel.getHand(HumanoidKinematicsToolboxController.BRACING_HAND_SIDE));
      if (isBracingHand)
      {
         // Posture adjustment
         FixedFrameQuaternionBasics desiredHandOrientation = command.getDesiredPose().getOrientation();
         desiredHandOrientation.append(bracingHandOrientationOffsetCalculator.getOrientationOffset());

         // Contact point adjustment
         if (!isUpperBodyLoadBearing.getValue())
         {
            FixedFramePoint3DBasics desiredHandPosition = command.getDesiredPose().getPosition();
            desiredHandPosition.add(integratedContactPointAdjustment);
         }
      }
   }

   public void addOrientationObjectives(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!KSTStreamingState.USE_DEFAULT_ORIENTATION_OBJECTIVES)
      {
         OrientationFeedbackControlCommand pelvisOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         pelvisOrientationCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
         pelvisOrientationCommand.setWeightsForSolver(pelvisWeight);
         pelvisOrientationCommand.setSelectionMatrix(pelvisSelectionMatrix);
         pelvisOrientationCommand.setGains(orientationGains);
         pelvisOrientationCommand.setInverseKinematics(pelvisOrientationCalculator.getDesiredOrientation(), null);
         pelvisOrientationCommand.getReferenceOrientation().append(pelvisOrientationOffsetCalculator.getOrientationOffset());

         OrientationFeedbackControlCommand chestOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         chestOrientationCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getChest());
         chestOrientationCommand.setWeightsForSolver(chestWeight);
         chestOrientationCommand.setSelectionMatrix(chestSelectionMatrix);
         chestOrientationCommand.setGains(orientationGains);
         chestOrientationCommand.setInverseKinematics(chestOrientationCalculator.getDesiredOrientation(), null);
         chestOrientationCommand.getReferenceOrientation().append(chestOrientationOffsetCalculator.getOrientationOffset());
      }

//      OneDoFJointFeedbackControlCommand spineXCommand = bufferToPack.addOneDoFJointFeedbackControlCommand();
//      spineXCommand.setJoint(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL));
//      spineXCommand.getGains().setKp(1200.0);
//      spineXCommand.setWeightForSolver(20.0);
//      spineXCommand.setInverseKinematics(0.0, 0.0);
//
//      OneDoFJointFeedbackControlCommand spineYCommand = bufferToPack.addOneDoFJointFeedbackControlCommand();
//      spineYCommand.setJoint(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH));
//      spineYCommand.getGains().setKp(1200.0);
//      spineYCommand.setWeightForSolver(20.0);
//      spineYCommand.setInverseKinematics(0.0, 0.0);
   }

   public void retargetCenterOfMass(KinematicsToolboxCenterOfMassCommand command)
   {
      command.getDesiredPosition().addZ(comHeightOffsetCalculator.getOffset());
   }

   private double updateAlphaEnabled()
   {
      double stabilityMargin = multiContactRegionCalculator.getStabilityMargin();
      double deltaStabilityMargin = stabilityMargin - minStabilityMargin.getValue();
      double enabledFraction = EuclidCoreTools.clamp(1.0 - deltaStabilityMargin / (stabilityMarginThreshold.getValue() - minStabilityMargin.getValue()), 0.0, 1.0);
      alphaEnabled.set(enabledFraction);
      return alphaEnabled.getValue();
   }
}
