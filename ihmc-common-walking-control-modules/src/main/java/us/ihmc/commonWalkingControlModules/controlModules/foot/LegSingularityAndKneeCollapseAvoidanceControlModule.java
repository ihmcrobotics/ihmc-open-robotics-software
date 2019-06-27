package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class LegSingularityAndKneeCollapseAvoidanceControlModule
{
   private boolean visualize = true;
   private boolean moreVisualizers = true;

   private static final boolean USE_KNEE_MECHANICAL_LIMIT_AVOIDANCE_SWING = false; // Limit the swing foot motion according to the knee flexion limit.
   private static final boolean USE_HIP_MECHANICAL_LIMIT_AVOIDANCE_SWING = false; // Limit the swing foot motion according to the hip flexion limit.
   private static final boolean USE_UNREACHABLE_FOOTSTEP_CORRECTION = true; // Lower the CoM if a footstep is unreachable
   private static final boolean USE_UNREACHABLE_FOOTSTEP_CORRECTION_ON_POSITION = true; // When false, the module will correct only the velocity and acceleration of the CoM height.
   private static final boolean USE_COLLAPSE_AVOIDANCE = false; // Try to avoid the knee from collapsing by limiting how low the CoM can be

   private final BooleanParameter useSingularityAvoidanceInSwing;
   private final BooleanParameter useSingularityAvoidanceInSupport;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame virtualLegTangentialFrameHipCentered, virtualLegTangentialFrameAnkleCentered;

   private final YoVariableRegistry registry;

   private final YoBoolean checkVelocityForSwingSingularityAvoidance;

   private final YoDouble alphaSwingSingularityAvoidance;
   private final YoDouble alphaSwingKneeMechanicalLimitAvoidance;
   private final YoDouble alphaSwingHipMechanicalLimitAvoidance;
   private final YoDouble alphaSupportSingularityAvoidance;
   private final YoDouble alphaCollapseAvoidance;
   private final YoDouble alphaUnreachableFootstep;

   private final YoDouble maximumLegLength;
   private final DoubleParameter minimumLegLength;

   private final YoDouble percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance;
   private final YoDouble hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance;
   private final YoDouble hipFlexionMechanicalLimit;
   private final YoDouble percentOfLegLengthThresholdToEnableSingularityAvoidance;
   private final YoDouble percentOfLegLengthThresholdToDisableSingularityAvoidance;
   private final YoDouble percentOfLegLengthThresholdForCollapseAvoidance;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSwing;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSupport;
   private final YoDouble minPercentOfLegLengthForCollapseAvoidance;
   private final YoDouble minMechanicalPercentOfLegLength;

   private final YoDouble footLoadThresholdToEnableCollapseAvoidance;
   private final YoDouble footLoadThresholdToDisableCollapseAvoidance;
   private final YoDouble timeDelayToDisableCollapseAvoidance;
   private final YoDouble timeSwitchCollapseAvoidance;
   private final YoDouble timeRemainingToDisableCollapseAvoidance;
   private final YoDouble yoTime;
   private final YoDouble timeSwitchSingularityAvoidance;

   private final YoDouble desiredPercentOfLegLength;
   private final YoDouble currentPercentOfLegLength;
   private final YoDouble correctedDesiredPercentOfLegLength;

   private final YoDouble desiredLegLength;
   private final YoDouble currentLegLength;
   private final YoDouble correctedDesiredLegLength;

   private final RigidBodyBasics pelvis;

   private final FrameVector3D unachievedSwingTranslationTemp = new FrameVector3D();
   private final FrameVector3D unachievedSwingVelocityTemp = new FrameVector3D();
   private final FrameVector3D unachievedSwingAccelerationTemp = new FrameVector3D();

   private final YoFrameVector3D unachievedSwingTranslation;
   private final YoFrameVector3D unachievedSwingVelocity;
   private final YoFrameVector3D unachievedSwingAcceleration;

   private final FramePoint3D desiredCenterOfMassHeightPoint = new FramePoint3D(worldFrame);
   private final FramePoint3D anklePosition = new FramePoint3D(worldFrame);
   private final FrameVector3D equivalentDesiredHipPitchHeightTranslation = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipVelocity = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipPitchAcceleration = new FrameVector3D();

   private final ReferenceFrame frameBeforeHipPitchJoint;

   private final Twist pelvisTwist = new Twist();
   private final FrameVector3D pelvisLinearVelocity = new FrameVector3D();
   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredFootLinearAcceleration = new FrameVector3D();

   private final YoFramePoint3D yoCurrentFootPosition;
   private final YoFramePoint3D yoDesiredFootPosition;
   private final YoFramePoint3D yoCorrectedDesiredFootPosition;

   private final YoFrameVector3D yoDesiredFootLinearVelocity;
   private final YoFrameVector3D yoCorrectedDesiredFootLinearVelocity;

   private final YoGraphicReferenceFrame virtualLegTangentialFrameHipCenteredGraphics, virtualLegTangentialFrameAnkleCenteredGraphics;
   private final YoGraphicPosition yoDesiredFootPositionGraphic, yoCorrectedDesiredFootPositionGraphic;
   private final YoGraphicVector yoDesiredFootLinearVelocityGraphic, yoCorrectedDesiredFootLinearVelocityGraphic;

   private final YoBoolean isSwingMechanicalLimitAvoidanceUsed;
   private final YoBoolean isSwingSingularityAvoidanceUsed;
   private final YoBoolean isSupportSingularityAvoidanceUsed;
   private final YoBoolean isSupportCollapseAvoidanceUsed;
   private final YoBoolean isUnreachableFootstepCompensated;
   private final YoBoolean doSmoothTransitionOutOfCollapseAvoidance;
   private final YoBoolean doSmoothTransitionOutOfSingularityAvoidance;

   private final YoDouble correctionAlphaFilter;
   private final AlphaFilteredYoVariable heightCorrectedFilteredForCollapseAvoidance;
   private final AlphaFilteredYoVariable heightVelocityCorrectedFilteredForCollapseAvoidance;
   private final AlphaFilteredYoVariable heightAcceleretionCorrectedFilteredForCollapseAvoidance;

   private final AlphaFilteredYoVariable heightCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightVelocityCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightAcceleretionCorrectedFilteredForSingularityAvoidance;

   private final YoDouble yoUnachievedSwingTranslation;
   private final AlphaFilteredYoVariable unachievedSwingTranslationFiltered;
   private final AlphaFilteredYoVariable unachievedSwingVelocityFiltered;
   private final AlphaFilteredYoVariable unachievedSwingAccelerationFiltered;

   private final double controlDT;
   private final OneDoFJointBasics hipPitchJoint;

   public LegSingularityAndKneeCollapseAvoidanceControlModule(String namePrefix, ContactablePlaneBody contactablePlaneBody, final RobotSide robotSide,
         WalkingControllerParameters walkingControllerParameters, final HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      unachievedSwingTranslation = new YoFrameVector3D("unachievedSwingTranslation", ReferenceFrame.getWorldFrame(), registry);
      unachievedSwingVelocity = new YoFrameVector3D("unachievedSwingVelocity", ReferenceFrame.getWorldFrame(), registry);
      unachievedSwingAcceleration = new YoFrameVector3D("unachievedSwingAcceleration", ReferenceFrame.getWorldFrame(), registry);

      maximumLegLength = new YoDouble(namePrefix + "MaxLegLength", registry);
      maximumLegLength.set(walkingControllerParameters.getMaximumLegLengthForSingularityAvoidance());

      minimumLegLength = new DoubleParameter(namePrefix + "MinLegLength", registry, walkingControllerParameters.getSwingTrajectoryParameters().getMinMechanicalLegLength());

      controlDT = controllerToolbox.getControlDT();
      yoTime = controllerToolbox.getYoTime();

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      hipPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH);
      frameBeforeHipPitchJoint = hipPitchJoint.getFrameBeforeJoint();
      endEffectorFrame = contactablePlaneBody.getFrameAfterParentJoint();

      checkVelocityForSwingSingularityAvoidance = new YoBoolean(namePrefix + "CheckVelocityForSwingSingularityAvoidance", registry);

      alphaSwingSingularityAvoidance = new YoDouble(namePrefix + "AlphaSwingSingularityAvoidance", registry);
      alphaSwingKneeMechanicalLimitAvoidance = new YoDouble(namePrefix + "AlphaSwingKneeMechanicalLimitAvoidance", registry);
      alphaSwingHipMechanicalLimitAvoidance = new YoDouble(namePrefix + "AlphaSwingHipMechanicalLimitAvoidance", registry);
      alphaSupportSingularityAvoidance = new YoDouble(namePrefix + "AlphaSupportSingularityAvoidance", registry);
      alphaCollapseAvoidance = new YoDouble(namePrefix + "AlphaCollapseAvoidance", registry);
      alphaUnreachableFootstep = new YoDouble(namePrefix + "AlphaUnreachableFootstep", registry);
      alphaUnreachableFootstep.set(0.25);

      yoUnachievedSwingTranslation = new YoDouble(namePrefix + "UnachievedSwingTranslation", registry);
      unachievedSwingTranslationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingTranslationFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingVelocityFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingVelocityFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingAccelerationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingAccelerationFiltered", registry, alphaUnreachableFootstep);

      percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance = new YoDouble(namePrefix + "PercThresSwingKneeLimitAvoidance", registry);
      hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance = new YoDouble(namePrefix + "ThresholdToEnableSwingHipLimitAvoidance", registry);
      hipFlexionMechanicalLimit = new YoDouble(namePrefix + "HipFlexionMechanicalLimit", registry);
      percentOfLegLengthThresholdToEnableSingularityAvoidance = new YoDouble(namePrefix + "PercThresSingularityAvoidance", registry);
      percentOfLegLengthThresholdToDisableSingularityAvoidance = new YoDouble(namePrefix + "PercThresToDisableSingularityAvoidance", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwing", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSupport = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSupport", registry);
      percentOfLegLengthThresholdForCollapseAvoidance = new YoDouble(namePrefix + "PercThresCollapseAvoidance", registry);
      minPercentOfLegLengthForCollapseAvoidance = new YoDouble(namePrefix + "MinPercOfLegLengthForCollapseAvoidance", registry);
      minMechanicalPercentOfLegLength = new YoDouble(namePrefix + "MinMechanicalPercOfLegLength", registry);
      footLoadThresholdToEnableCollapseAvoidance = new YoDouble(namePrefix + "LoadThresholdToEnableCollapseAvoidance", registry);
      footLoadThresholdToDisableCollapseAvoidance = new YoDouble(namePrefix + "LoadThresholdToDisableCollapseAvoidance", registry);
      timeDelayToDisableCollapseAvoidance = new YoDouble(namePrefix + "TimeDelayToDisableCollapseAvoidance", registry);
      timeSwitchCollapseAvoidance = new YoDouble(namePrefix + "TimeSwitchCollapseAvoidance", registry);
      timeRemainingToDisableCollapseAvoidance = new YoDouble(namePrefix + "TimeRemainingToDisableCollapseAvoidance", registry);
      timeSwitchSingularityAvoidance = new YoDouble(namePrefix + "TimeSwitchSingularityAvoidance", registry);

      percentOfLegLengthThresholdToEnableSingularityAvoidance.set(0.87);
      percentOfLegLengthThresholdToDisableSingularityAvoidance.set(0.85);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing.set(0.97);
      maxPercentOfLegLengthForSingularityAvoidanceInSupport.set(0.98);
      percentOfLegLengthThresholdForCollapseAvoidance.set(0.83);
      minPercentOfLegLengthForCollapseAvoidance.set(0.76);//walkingControllerParameters.getMinLegLengthBeforeCollapsingSingleSupport() / maximumLegLength.getDoubleValue());
      footLoadThresholdToEnableCollapseAvoidance.set(0.62); // 0.65
      footLoadThresholdToDisableCollapseAvoidance.set(0.59); // 0.62
      timeDelayToDisableCollapseAvoidance.set(0.5);
      timeRemainingToDisableCollapseAvoidance.set(timeDelayToDisableCollapseAvoidance.getDoubleValue());
      timeSwitchCollapseAvoidance.set(yoTime.getDoubleValue());

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      useSingularityAvoidanceInSwing = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSwing", registry, swingTrajectoryParameters.useSingularityAvoidanceInSwing());
      useSingularityAvoidanceInSupport = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSupport", registry, swingTrajectoryParameters.useSingularityAvoidanceInSupport());

      double deltaAwayFromLimit = Math.toRadians(20.0);
      if (hipPitchJoint.getJointAxis().getY() > 0.0)
      {
         hipFlexionMechanicalLimit.set(hipPitchJoint.getJointLimitLower());
         hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance.set(hipPitchJoint.getJointLimitLower() + deltaAwayFromLimit);
      }
      else
      {
         hipFlexionMechanicalLimit.set(hipPitchJoint.getJointLimitUpper());
         hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance.set(hipPitchJoint.getJointLimitUpper() - deltaAwayFromLimit);
      }

      correctionAlphaFilter = new YoDouble(namePrefix + "CorrectionAlphaFilter", registry);
      heightCorrectedFilteredForCollapseAvoidance = new AlphaFilteredYoVariable(namePrefix + "HeightCorrectedFilteredForCollapseAvoidance", registry,
            correctionAlphaFilter);
      heightVelocityCorrectedFilteredForCollapseAvoidance = new AlphaFilteredYoVariable(namePrefix + "HeightVelocityCorrectedFilteredForCollapseAvoidance",
            registry, correctionAlphaFilter);
      heightAcceleretionCorrectedFilteredForCollapseAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightAcceleretionCorrectedFilteredForCollapseAvoidance", registry, correctionAlphaFilter);

      heightCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(namePrefix + "HeightCorrectedFilteredForSingularityAvoidance", registry,
            correctionAlphaFilter);
      heightVelocityCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightVelocityCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);
      heightAcceleretionCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightAcceleretionCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);

      correctionAlphaFilter.set(0.98);

      desiredPercentOfLegLength = new YoDouble(namePrefix + "DesiredPercentOfLegLength", registry);
      correctedDesiredPercentOfLegLength = new YoDouble(namePrefix + "CorrectedDesiredPercentOfLegLength", registry);
      currentPercentOfLegLength = new YoDouble(namePrefix + "CurrentPercentOfLegLength", registry);

      desiredLegLength = new YoDouble(namePrefix + "DesiredLegLength", registry);
      correctedDesiredLegLength = new YoDouble(namePrefix + "CorrectedDesiredLegLength", registry);
      currentLegLength = new YoDouble(namePrefix + "CurrentLegLength", registry);

      isSwingMechanicalLimitAvoidanceUsed = new YoBoolean(namePrefix + "IsSwingMechanicalLimitAvoidanceUsed", registry);
      isSwingSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSwingSingularityAvoidanceUsed", registry);
      isSupportSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSupportSingularityAvoidanceUsed", registry);
      isSupportCollapseAvoidanceUsed = new YoBoolean(namePrefix + "IsSupportCollapseAvoidanceUsed", registry);
      isUnreachableFootstepCompensated = new YoBoolean(namePrefix + "IsUnreachableFootstepCompensated", registry);
      doSmoothTransitionOutOfCollapseAvoidance = new YoBoolean(namePrefix + "DoSmoothTransitionCollapseAvoidance", registry);
      doSmoothTransitionOutOfSingularityAvoidance = new YoBoolean(namePrefix + "DoSmoothTransitionSingularityAvoidance", registry);

      final ReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
      virtualLegTangentialFrameHipCentered = new ReferenceFrame(namePrefix + "VirtualLegTangentialFrameHipCentered", pelvisFrame)
      {
         private final AxisAngle hipPitchRotationToParentFrame = new AxisAngle();
         private final FramePoint3D tempPoint = new FramePoint3D();
         private final FrameVector3D footToHipAxis = new FrameVector3D();
         private final FramePoint3D hipPitchPosition = new FramePoint3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            tempPoint.setToZero(frameBeforeHipPitchJoint);
            tempPoint.changeFrame(endEffectorFrame);
            footToHipAxis.setIncludingFrame(tempPoint);
            footToHipAxis.changeFrame(getParent());
            EuclidGeometryTools.axisAngleFromZUpToVector3D(footToHipAxis, hipPitchRotationToParentFrame);
            hipPitchPosition.setToZero(frameBeforeHipPitchJoint);
            hipPitchPosition.changeFrame(getParent());

            transformToParent.setRotationAndZeroTranslation(hipPitchRotationToParentFrame);
            transformToParent.setTranslation(hipPitchPosition);
         }
      };

      virtualLegTangentialFrameAnkleCentered = new ReferenceFrame(namePrefix + "VirtualLegTangentialFrameAnkleCentered", pelvisFrame)
      {
         private final AxisAngle anklePitchRotationToParentFrame = new AxisAngle();
         private final FramePoint3D tempPoint = new FramePoint3D();
         private final FrameVector3D footToHipAxis = new FrameVector3D();
         private final FramePoint3D anklePitchPosition = new FramePoint3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            tempPoint.setToZero(frameBeforeHipPitchJoint);
            tempPoint.changeFrame(endEffectorFrame);
            footToHipAxis.setIncludingFrame(tempPoint);
            footToHipAxis.changeFrame(getParent());
            EuclidGeometryTools.axisAngleFromZUpToVector3D(footToHipAxis, anklePitchRotationToParentFrame);
            anklePitchPosition.setToZero(endEffectorFrame);
            anklePitchPosition.changeFrame(getParent());

            transformToParent.setRotationAndZeroTranslation(anklePitchRotationToParentFrame);
            transformToParent.setTranslation(anklePitchPosition);
         }
      };

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      visualize = visualize && yoGraphicsListRegistry != null;
      moreVisualizers = visualize && moreVisualizers;

      yoCurrentFootPosition = new YoFramePoint3D(namePrefix + "CurrentFootPosition", worldFrame, registry);
      yoDesiredFootPosition = new YoFramePoint3D(namePrefix + "DesiredFootPosition", worldFrame, registry);
      yoCorrectedDesiredFootPosition = new YoFramePoint3D(namePrefix + "CorrectedDesiredFootPosition", worldFrame, registry);
      yoDesiredFootLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredFootLinearVelocity", worldFrame, registry);
      yoCorrectedDesiredFootLinearVelocity = new YoFrameVector3D(namePrefix + "CorrectedDesiredFootLinearVelocity", worldFrame, registry);
      yoDesiredFootPosition.setToNaN();
      yoCorrectedDesiredFootPosition.setToNaN();
      yoDesiredFootLinearVelocity.setToNaN();
      yoCorrectedDesiredFootLinearVelocity.setToNaN();

      if (visualize)
      {
         yoDesiredFootPositionGraphic = new YoGraphicPosition(namePrefix + "DesiredFootPosition", yoDesiredFootPosition, 0.025, YoAppearance.Red(),
               GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootPositionGraphic);
         yoCorrectedDesiredFootPositionGraphic = new YoGraphicPosition(namePrefix + "CorrectedDesiredFootPosition", yoCorrectedDesiredFootPosition, 0.025,
               YoAppearance.Green(), GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoCorrectedDesiredFootPositionGraphic);
      }
      else
      {
         yoDesiredFootPositionGraphic = null;
         yoCorrectedDesiredFootPositionGraphic = null;
      }

      if (moreVisualizers)
      {
         virtualLegTangentialFrameHipCenteredGraphics = new YoGraphicReferenceFrame(virtualLegTangentialFrameHipCentered, registry, false, 0.1);
         virtualLegTangentialFrameAnkleCenteredGraphics = new YoGraphicReferenceFrame(virtualLegTangentialFrameAnkleCentered, registry, false, 0.1);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", virtualLegTangentialFrameHipCenteredGraphics);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", virtualLegTangentialFrameAnkleCenteredGraphics);

         yoDesiredFootLinearVelocityGraphic = new YoGraphicVector(namePrefix + "DesiredFootLinearVelocity", yoCurrentFootPosition, yoDesiredFootLinearVelocity,
               0.2, YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootLinearVelocityGraphic);
         yoCorrectedDesiredFootLinearVelocityGraphic = new YoGraphicVector(namePrefix + "CorrectedDesiredFootLinearVelocity", yoCurrentFootPosition,
               yoCorrectedDesiredFootLinearVelocity, 0.2, YoAppearance.Green());
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoCorrectedDesiredFootLinearVelocityGraphic);
      }
      else
      {
         virtualLegTangentialFrameHipCenteredGraphics = null;
         virtualLegTangentialFrameAnkleCenteredGraphics = null;
         yoDesiredFootLinearVelocityGraphic = null;
         yoCorrectedDesiredFootLinearVelocityGraphic = null;
      }
   }

   public void update()
   {
      virtualLegTangentialFrameHipCentered.update();
      virtualLegTangentialFrameAnkleCentered.update();

      anklePosition.setToZero(endEffectorFrame);
      yoCurrentFootPosition.setMatchingFrame(anklePosition);
      anklePosition.changeFrame(virtualLegTangentialFrameHipCentered);
      currentLegLength.set(-anklePosition.getZ());
   }

   public void resetHeightCorrectionParameters()
   {
      doSmoothTransitionOutOfCollapseAvoidance.set(false);
      doSmoothTransitionOutOfSingularityAvoidance.set(false);

      isSupportCollapseAvoidanceUsed.set(false);
      isSupportSingularityAvoidanceUsed.set(false);
   }

   public void resetSwingParameters()
   {
      yoDesiredFootPosition.setToNaN();
      yoCorrectedDesiredFootPosition.setToNaN();
      yoDesiredFootLinearVelocity.setToNaN();
      yoCorrectedDesiredFootLinearVelocity.setToNaN();

      if (visualize)
      {
         yoDesiredFootPositionGraphic.hideGraphicObject();
         yoCorrectedDesiredFootPositionGraphic.hideGraphicObject();
         if (moreVisualizers)
         {
            virtualLegTangentialFrameHipCenteredGraphics.update();
            virtualLegTangentialFrameAnkleCenteredGraphics.update();
            yoDesiredFootLinearVelocityGraphic.hideGraphicObject();
            yoCorrectedDesiredFootLinearVelocityGraphic.hideGraphicObject();
         }
      }

      alphaSwingSingularityAvoidance.set(0.0);
      unachievedSwingTranslation.setToZero();
      unachievedSwingVelocity.setToZero();
      unachievedSwingAcceleration.setToZero();
   }

   public void setCheckVelocityForSwingSingularityAvoidance(boolean value)
   {
      checkVelocityForSwingSingularityAvoidance.set(value);
   }

   public void correctSwingFootTrajectory(FramePoint3D desiredFootPositionToCorrect, FrameVector3D desiredFootLinearVelocityToCorrect,
         FrameVector3D desiredFootLinearAccelerationToCorrect)
   {
      isSwingSingularityAvoidanceUsed.set(false);
      alphaSwingSingularityAvoidance.set(0.0);

      isSwingMechanicalLimitAvoidanceUsed.set(false);
      alphaSwingKneeMechanicalLimitAvoidance.set(0.0);

      yoDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);

      anklePosition.setToZero(endEffectorFrame);
      anklePosition.changeFrame(worldFrame);
      yoCurrentFootPosition.set(anklePosition);
      anklePosition.changeFrame(virtualLegTangentialFrameHipCentered);
      currentLegLength.set(-anklePosition.getZ());
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      desiredFootPosition.setIncludingFrame(desiredFootPositionToCorrect);
      desiredFootLinearAcceleration.setIncludingFrame(desiredFootLinearAccelerationToCorrect);
      desiredFootLinearVelocity.setIncludingFrame(desiredFootLinearVelocityToCorrect);

      desiredFootPosition.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearAcceleration.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameHipCentered);

      desiredLegLength.set(-desiredFootPosition.getZ());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
      correctedDesiredLegLength.set(desiredLegLength.getDoubleValue());
      correctedDesiredPercentOfLegLength.set(desiredPercentOfLegLength.getDoubleValue());

      if (USE_KNEE_MECHANICAL_LIMIT_AVOIDANCE_SWING)
         correctSwingFootTrajectoryForMechanicalLimitAvoidance(desiredFootPositionToCorrect, desiredFootLinearVelocityToCorrect,
               desiredFootLinearAccelerationToCorrect);
      if (useSingularityAvoidanceInSwing.getValue())
         correctSwingFootTrajectoryForSingularityAvoidance(desiredFootPositionToCorrect, desiredFootLinearVelocityToCorrect,
               desiredFootLinearAccelerationToCorrect);
   }

   private void correctSwingFootTrajectoryForMechanicalLimitAvoidance(FramePoint3D desiredFootPositionToCorrect, FrameVector3D desiredFootLinearVelocityToCorrect,
         FrameVector3D desiredFootLinearAccelerationToCorrect)
   {
      minMechanicalPercentOfLegLength.set(minimumLegLength.getValue() / maximumLegLength.getDoubleValue());
      percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance.set(0.07 + minMechanicalPercentOfLegLength.getDoubleValue());

      if (desiredPercentOfLegLength.getDoubleValue() > percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance.getDoubleValue())
         return;

      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
      pelvis.getBodyFixedFrame().getTwistOfFrame(pelvisTwist);
      //      pelvisTwist.changeFrame(virtualLegTangentialFrameAnkleCentered);
      pelvisLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      pelvisLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      isSwingMechanicalLimitAvoidanceUsed.set(true);

      alphaSwingKneeMechanicalLimitAvoidance
            .set((desiredPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance.getDoubleValue())
                  / (minMechanicalPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdToEnableSwingKneeLimitAvoidance.getDoubleValue()));
      alphaSwingKneeMechanicalLimitAvoidance.set(MathTools.clamp(alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue(), 0.0, 1.0));

      //      double desiredOrMinLegLength = - Math.max(desiredLegLength.getDoubleValue(), minimumLegLength.getDoubleValue());
      double correctedDesiredPositionZ = ((1.0 - alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue()) * desiredLegLength.getDoubleValue()
            + alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue() * currentLegLength.getDoubleValue());
      correctedDesiredPositionZ = -Math.max(correctedDesiredPositionZ, minimumLegLength.getValue());
      //      unachievedSwingTranslation.setIncludingFrame(desiredFootPosition.getReferenceFrame(), 0.0, 0.0, desiredFootPosition.getZ() - correctedDesiredPositionZ);
      //      unachievedSwingTranslation.changeFrame(worldFrame);
      //      yoUnachievedSwingTranslation.set(unachievedSwingTranslation.getZ());
      desiredFootPosition.setZ(correctedDesiredPositionZ);

      correctedDesiredLegLength.set(Math.abs(correctedDesiredPositionZ));
      correctedDesiredPercentOfLegLength.set(correctedDesiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      //      if (desiredFootLinearVelocity.getZ() - pelvisLinearVelocity.getZ() < 0.0) // Check if desired velocity results in leg flexion
      {
         double desiredLinearVelocityX = desiredFootLinearVelocity.getX();
         double desiredLinearVelocityY = desiredFootLinearVelocity.getY();
         // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
         double desiredLinearVelocityZ = (1.0 - alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue()) * desiredFootLinearVelocity.getZ()
               + alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue() * pelvisLinearVelocity.getZ();

         //         unachievedSwingVelocity.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0, desiredFootLinearVelocity.getZ() - desiredLinearVelocityZ);
         desiredFootLinearVelocity.setIncludingFrame(virtualLegTangentialFrameAnkleCentered, desiredLinearVelocityX, desiredLinearVelocityY,
               desiredLinearVelocityZ);
      }
      //      desiredFootLinearVelocity.setToZero();
      //      if (desiredFootLinearAcceleration.getZ() < 0.0) // Check if desired acceleration results in leg extension
      {
         //         unachievedSwingAcceleration.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0, alphaSwingMechanicalLimitAvoidance.getDoubleValue() * desiredFootLinearAcceleration.getZ());
         desiredFootLinearAcceleration.setZ((1.0 - alphaSwingKneeMechanicalLimitAvoidance.getDoubleValue()) * desiredFootLinearAcceleration.getZ());
      }
      //      desiredFootLinearAcceleration.setToZero();

      if (USE_HIP_MECHANICAL_LIMIT_AVOIDANCE_SWING)
      {
         alphaSwingHipMechanicalLimitAvoidance.set((hipPitchJoint.getQ() - hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance.getDoubleValue())
               / (hipFlexionMechanicalLimit.getDoubleValue() - hipFlexionAngleThresholdToEnableSwingHipLimitAvoidance.getDoubleValue()));
         alphaSwingHipMechanicalLimitAvoidance.set(MathTools.clamp(alphaSwingHipMechanicalLimitAvoidance.getDoubleValue(), 0.0, 1.0));

         if (alphaSwingHipMechanicalLimitAvoidance.getDoubleValue() > 1.0e-3)
         {
            double correctedDesiredPositionX = desiredFootPosition.getX();
            if (correctedDesiredPositionX > 0.0)
            {
               correctedDesiredPositionX *= 1.0 - alphaSwingHipMechanicalLimitAvoidance.getDoubleValue();
               desiredFootPosition.setX(correctedDesiredPositionX);
            }

            double correctedDesiredLinearVelocityX = desiredFootLinearVelocity.getX();
            if (correctedDesiredLinearVelocityX > 0.0)
            {
               correctedDesiredLinearVelocityX = (1.0 - alphaSwingHipMechanicalLimitAvoidance.getDoubleValue()) * correctedDesiredLinearVelocityX
                     + alphaSwingHipMechanicalLimitAvoidance.getDoubleValue() * pelvisLinearVelocity.getX();
               desiredFootLinearVelocity.setX(correctedDesiredLinearVelocityX);
            }

            double correctedDesiredLinearAccelerationX = desiredFootLinearAcceleration.getX();
            if (correctedDesiredLinearAccelerationX > 0.0)
            {
               correctedDesiredLinearAccelerationX *= 1.0 - alphaSwingHipMechanicalLimitAvoidance.getDoubleValue();
               desiredFootLinearAcceleration.setX(correctedDesiredLinearAccelerationX);
            }
         }
      }

      desiredFootPosition.changeFrame(desiredFootPositionToCorrect.getReferenceFrame());
      desiredFootLinearVelocity.changeFrame(desiredFootLinearVelocityToCorrect.getReferenceFrame());
      desiredFootLinearAcceleration.changeFrame(desiredFootLinearAccelerationToCorrect.getReferenceFrame());

      desiredFootPositionToCorrect.set(desiredFootPosition);
      desiredFootLinearVelocityToCorrect.set(desiredFootLinearVelocity);
      desiredFootLinearAccelerationToCorrect.set(desiredFootLinearAcceleration);

      yoCorrectedDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoCorrectedDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);

      if (visualize)
      {
         yoDesiredFootPositionGraphic.showGraphicObject();
         yoCorrectedDesiredFootPositionGraphic.showGraphicObject();
         if (moreVisualizers)
         {
            yoDesiredFootLinearVelocityGraphic.showGraphicObject();
            yoCorrectedDesiredFootLinearVelocityGraphic.showGraphicObject();
         }
      }
   }

   private void correctSwingFootTrajectoryForSingularityAvoidance(FramePoint3D desiredFootPositionToCorrect, FrameVector3D desiredFootLinearVelocityToCorrect,
         FrameVector3D desiredFootLinearAccelerationToCorrect)
   {
      if (desiredPercentOfLegLength.getDoubleValue() < percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue())
         return;

      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
      pelvis.getBodyFixedFrame().getTwistOfFrame(pelvisTwist);
      pelvisLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      pelvisLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && (desiredFootLinearVelocity.getZ() - pelvisLinearVelocity.getZ() > -1e-10))
         return;

      checkVelocityForSwingSingularityAvoidance.set(false);
      isSwingSingularityAvoidanceUsed.set(true);

      alphaSwingSingularityAvoidance.set((desiredPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue())
            / (maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue()
                  - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue()));
      alphaSwingSingularityAvoidance.set(MathTools.clamp(alphaSwingSingularityAvoidance.getDoubleValue(), 0.0, 1.0));

      double desiredOrMaxLegLength = -Math.min(desiredLegLength.getDoubleValue(),
            maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() * maximumLegLength.getDoubleValue());
      double correctedDesiredPositionZ = desiredOrMaxLegLength; //(1.0 - alphaSingularityAvoidance.getDoubleValue()) * desiredFootPosition.getZ() + alphaSingularityAvoidance.getDoubleValue() * desiredOrMaxLegLength;

      unachievedSwingTranslationTemp.setIncludingFrame(desiredFootPosition.getReferenceFrame(), 0.0, 0.0,
            desiredFootPosition.getZ() - correctedDesiredPositionZ);
      unachievedSwingTranslationTemp.changeFrame(worldFrame);
      unachievedSwingTranslation.set(unachievedSwingTranslationTemp);

      yoUnachievedSwingTranslation.set(unachievedSwingTranslation.getZ());
      desiredFootPosition.setZ(correctedDesiredPositionZ);

      correctedDesiredLegLength.set(Math.abs(correctedDesiredPositionZ));
      correctedDesiredPercentOfLegLength.set(correctedDesiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      //      if (desiredFootLinearVelocity.getZ() - pelvisLinearVelocity.getZ() < 0.0) // Check if desired velocity results in leg extension
      {
         double desiredLinearVelocityX = desiredFootLinearVelocity.getX();
         double desiredLinearVelocityY = desiredFootLinearVelocity.getY();
         // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
         double desiredLinearVelocityZ = (1.0 - alphaSwingSingularityAvoidance.getDoubleValue()) * desiredFootLinearVelocity.getZ()
               + alphaSwingSingularityAvoidance.getDoubleValue() * pelvisLinearVelocity.getZ();

         unachievedSwingVelocityTemp.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0,
               desiredFootLinearVelocity.getZ() - desiredLinearVelocityZ);
         unachievedSwingVelocityTemp.changeFrame(worldFrame);
         unachievedSwingVelocity.set(unachievedSwingVelocityTemp);

         desiredFootLinearVelocity.setIncludingFrame(virtualLegTangentialFrameAnkleCentered, desiredLinearVelocityX, desiredLinearVelocityY,
               desiredLinearVelocityZ);
      }

      //      if (desiredFootLinearAcceleration.getZ() < 0.0) // Check if desired acceleration results in leg extension
      {
         unachievedSwingAccelerationTemp.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0,
               alphaSwingSingularityAvoidance.getDoubleValue() * desiredFootLinearAcceleration.getZ());
         unachievedSwingAccelerationTemp.changeFrame(worldFrame);
         unachievedSwingAcceleration.set(unachievedSwingAccelerationTemp);

         desiredFootLinearAcceleration.setZ((1.0 - alphaSwingSingularityAvoidance.getDoubleValue()) * desiredFootLinearAcceleration.getZ());
      }

      desiredFootPosition.changeFrame(desiredFootPositionToCorrect.getReferenceFrame());
      desiredFootLinearVelocity.changeFrame(desiredFootLinearVelocityToCorrect.getReferenceFrame());
      desiredFootLinearAcceleration.changeFrame(desiredFootLinearAccelerationToCorrect.getReferenceFrame());

      desiredFootPositionToCorrect.set(desiredFootPosition);
      desiredFootLinearVelocityToCorrect.set(desiredFootLinearVelocity);
      desiredFootLinearAccelerationToCorrect.set(desiredFootLinearAcceleration);

      yoCorrectedDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoCorrectedDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);

      if (visualize)
      {
         yoDesiredFootPositionGraphic.showGraphicObject();
         yoCorrectedDesiredFootPositionGraphic.showGraphicObject();
         if (moreVisualizers)
         {
            yoDesiredFootLinearVelocityGraphic.showGraphicObject();
            yoCorrectedDesiredFootLinearVelocityGraphic.showGraphicObject();
         }
      }
   }

   public void correctCoMHeightTrajectoryForSupportLeg(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
         ReferenceFrame pelvisZUpFrame, double footLoadPercentage, ConstraintType constraintType)
   {
      correctCoMHeightTrajectoryForSingularityAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent, pelvisZUpFrame, constraintType);

      if (!isSupportSingularityAvoidanceUsed.getBooleanValue())
         correctCoMHeightTrajectoryForCollapseAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent, pelvisZUpFrame, footLoadPercentage, constraintType);
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect,
         double zCurrent, ReferenceFrame pelvisZUpFrame, ConstraintType constraintType)
   {
      if (!useSingularityAvoidanceInSupport.getValue())
      {
         alphaSupportSingularityAvoidance.set(0.0);
         isSupportSingularityAvoidanceUsed.set(false);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);
         return;
      }

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);
      equivalentDesiredHipPitchHeightTranslation.setIncludingFrame(worldFrame, 0.0, 0.0, desiredCenterOfMassHeightPoint.getZ() - zCurrent);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(virtualLegTangentialFrameAnkleCentered);

      equivalentDesiredHipVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightVelocity());
      equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
      comXYVelocity.changeFrame(pelvisZUpFrame);
      equivalentDesiredHipVelocity.setX(comXYVelocity.getX());
      equivalentDesiredHipVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      equivalentDesiredHipPitchAcceleration.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightAcceleration());
      equivalentDesiredHipPitchAcceleration.changeFrame(virtualLegTangentialFrameAnkleCentered);

      desiredLegLength.set(equivalentDesiredHipPitchHeightTranslation.getZ() + currentLegLength.getDoubleValue());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
      correctedDesiredLegLength.set(desiredLegLength.getDoubleValue());
      correctedDesiredPercentOfLegLength.set(desiredPercentOfLegLength.getDoubleValue());

      if (constraintType != ConstraintType.FULL)
      {
         alphaSupportSingularityAvoidance.set(0.0);
         doSmoothTransitionOutOfSingularityAvoidance.set(isSupportSingularityAvoidanceUsed.getBooleanValue());
         if (!isSupportSingularityAvoidanceUsed.getBooleanValue())
            return;
      }

      if (isSupportSingularityAvoidanceUsed.getBooleanValue() || doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
      {
         if (desiredPercentOfLegLength.getDoubleValue() < percentOfLegLengthThresholdToDisableSingularityAvoidance.getDoubleValue())
         {
            if (!doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
            {
               alphaSupportSingularityAvoidance.set(0.0);
               doSmoothTransitionOutOfSingularityAvoidance.set(true);
            }
         }
      }

      // Check if the leg is extended such as the trajectory needs to be corrected
      if (desiredPercentOfLegLength.getDoubleValue() < percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue())
      {
         if (!isSupportSingularityAvoidanceUsed.getBooleanValue() && !doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
            return;
      }
      else if (!isSupportSingularityAvoidanceUsed.getBooleanValue())
      {
         isSupportSingularityAvoidanceUsed.set(true);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);
         timeSwitchSingularityAvoidance.set(yoTime.getDoubleValue());

         heightCorrectedFilteredForSingularityAvoidance.reset();
         heightVelocityCorrectedFilteredForSingularityAvoidance.reset();
         heightAcceleretionCorrectedFilteredForSingularityAvoidance.reset();
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightVelocityCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
         heightAcceleretionCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());
      }

      if (doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
      {
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightVelocityCorrectedFilteredForSingularityAvoidance.set(comHeightDataToCorrect.getComHeightVelocity());
         heightAcceleretionCorrectedFilteredForSingularityAvoidance.set(comHeightDataToCorrect.getComHeightAcceleration());

         comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(),
               heightCorrectedFilteredForSingularityAvoidance.getDoubleValue());
         comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForSingularityAvoidance.getDoubleValue());
         comHeightDataToCorrect.setComHeightAcceleration(heightAcceleretionCorrectedFilteredForSingularityAvoidance.getDoubleValue());

         // If the filtered desired com height caught up to the input one, then stop doing smooth transition.
         if (Math.abs(desiredCenterOfMassHeightPoint.getZ() - heightCorrectedFilteredForSingularityAvoidance.getDoubleValue()) <= 5e-3)
         {
            alphaSupportSingularityAvoidance.set(0.0);
            isSupportSingularityAvoidanceUsed.set(false);
            doSmoothTransitionOutOfSingularityAvoidance.set(false);
         }

         // If height is lower than filtered and the knee is bent enough, then really want to get out of singularity avoidance faster. So in this case, smooth faster...            
         else if (desiredCenterOfMassHeightPoint.getZ() <= heightCorrectedFilteredForSingularityAvoidance.getDoubleValue() && 
               (desiredPercentOfLegLength.getDoubleValue() < percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue()))
         {
            heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
            heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());

            // If leg is bent a lot and singularity avoidance no longer needed, stop smoothing...
            if (desiredPercentOfLegLength.getDoubleValue() < percentOfLegLengthThresholdToDisableSingularityAvoidance.getDoubleValue() - 0.05)
            {
               alphaSupportSingularityAvoidance.set(0.0);
               isSupportSingularityAvoidanceUsed.set(false);
               doSmoothTransitionOutOfSingularityAvoidance.set(false);
            }
         }
         return;
      }

      anklePosition.setToZero(endEffectorFrame);
      anklePosition.changeFrame(worldFrame);
      yoCurrentFootPosition.set(anklePosition);
      anklePosition.changeFrame(virtualLegTangentialFrameHipCentered);
      currentLegLength.set(-anklePosition.getZ());
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      alphaSupportSingularityAvoidance
            .set((desiredPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue())
                  / (maxPercentOfLegLengthForSingularityAvoidanceInSupport.getDoubleValue()
                        - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue()));
      alphaSupportSingularityAvoidance.set(MathTools.clamp(alphaSupportSingularityAvoidance.getDoubleValue(), 0.0, 1.0));

      double desiredOrMaxLegLength = Math.min(desiredLegLength.getDoubleValue(),
            maxPercentOfLegLengthForSingularityAvoidanceInSupport.getDoubleValue() * maximumLegLength.getDoubleValue());
      double correctedDesiredTranslationZ = desiredOrMaxLegLength - currentLegLength.getDoubleValue();
      //      double correctedDesiredTranslationZ = (1.0 - alphaSingularityAvoidance.getDoubleValue()) * equivalentDesiredHipPitchHeightTranslation.getZ() + alphaSingularityAvoidance.getDoubleValue() * (desiredOrMaxLegLength - currentLegLength.getDoubleValue());
      equivalentDesiredHipPitchHeightTranslation.setZ(correctedDesiredTranslationZ);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(worldFrame);

      correctedDesiredLegLength.set(correctedDesiredTranslationZ + currentLegLength.getDoubleValue());
      correctedDesiredPercentOfLegLength.set(correctedDesiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      desiredCenterOfMassHeightPoint.setZ(zCurrent + equivalentDesiredHipPitchHeightTranslation.getZ());
      heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
      comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForSingularityAvoidance.getDoubleValue());

      //      if (equivalentDesiredHipVelocity.getZ() > 0.0) // Check if desired velocity results in leg extension
      {
         equivalentDesiredHipVelocity.setZ((1.0 - alphaSupportSingularityAvoidance.getDoubleValue()) * equivalentDesiredHipVelocity.getZ());
         equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
         if (Math.abs(comXYVelocity.getX()) > 1e-3 && Math.abs(equivalentDesiredHipVelocity.getX()) > 1e-3)
            equivalentDesiredHipVelocity.scale(comXYVelocity.getX() / equivalentDesiredHipVelocity.getX());
         equivalentDesiredHipVelocity.changeFrame(worldFrame);
         heightVelocityCorrectedFilteredForSingularityAvoidance.update(equivalentDesiredHipVelocity.getZ());
         comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      }

      //      if (equivalentDesiredHipPitchAcceleration.getZ() > 0.0) // Check if desired acceleration results in leg extension
      {
         equivalentDesiredHipPitchAcceleration.setZ((1.0 - alphaSupportSingularityAvoidance.getDoubleValue()) * equivalentDesiredHipPitchAcceleration.getZ());
         equivalentDesiredHipPitchAcceleration.changeFrame(worldFrame);
         heightAcceleretionCorrectedFilteredForSingularityAvoidance.update(equivalentDesiredHipPitchAcceleration.getZ());
         comHeightDataToCorrect.setComHeightAcceleration(heightAcceleretionCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      }
   }

   public void correctCoMHeightTrajectoryForCollapseAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
         ReferenceFrame pelvisZUpFrame, double footLoadPercentage, ConstraintType constraintType)
   {
      if (!USE_COLLAPSE_AVOIDANCE)
      {
         alphaCollapseAvoidance.set(0.0);
         isSupportCollapseAvoidanceUsed.set(false);
         doSmoothTransitionOutOfCollapseAvoidance.set(false);
         return;
      }

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);
      equivalentDesiredHipPitchHeightTranslation.setIncludingFrame(worldFrame, 0.0, 0.0, desiredCenterOfMassHeightPoint.getZ() - zCurrent);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(virtualLegTangentialFrameAnkleCentered);

      equivalentDesiredHipVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightVelocity());
      equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
      comXYVelocity.changeFrame(pelvisZUpFrame);
      equivalentDesiredHipVelocity.setX(comXYVelocity.getX());
      equivalentDesiredHipVelocity.changeFrame(worldFrame);
      equivalentDesiredHipVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      equivalentDesiredHipPitchAcceleration.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightAcceleration());
      equivalentDesiredHipPitchAcceleration.changeFrame(virtualLegTangentialFrameAnkleCentered);

      desiredLegLength.set(equivalentDesiredHipPitchHeightTranslation.getZ() + currentLegLength.getDoubleValue());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
      correctedDesiredLegLength.set(desiredLegLength.getDoubleValue());
      correctedDesiredPercentOfLegLength.set(desiredPercentOfLegLength.getDoubleValue());

      if (constraintType != ConstraintType.FULL)
      {
         alphaCollapseAvoidance.set(0.0);
         doSmoothTransitionOutOfCollapseAvoidance.set(isSupportCollapseAvoidanceUsed.getBooleanValue());
         timeRemainingToDisableCollapseAvoidance.set(0.0);
         if (!isSupportCollapseAvoidanceUsed.getBooleanValue())
            return;
      }

      if (!isSupportCollapseAvoidanceUsed.getBooleanValue() && !doSmoothTransitionOutOfCollapseAvoidance.getBooleanValue())
      {
         if (footLoadPercentage < footLoadThresholdToEnableCollapseAvoidance.getDoubleValue())
            return;
      }
      else if (footLoadPercentage < footLoadThresholdToDisableCollapseAvoidance.getDoubleValue())
      {
         timeRemainingToDisableCollapseAvoidance.sub(controlDT);
         timeRemainingToDisableCollapseAvoidance.set(Math.max(0.0, timeRemainingToDisableCollapseAvoidance.getDoubleValue()));
         if (timeRemainingToDisableCollapseAvoidance.getDoubleValue() <= 0.0 && !doSmoothTransitionOutOfCollapseAvoidance.getBooleanValue())
         {
            alphaCollapseAvoidance.set(0.0);
            doSmoothTransitionOutOfCollapseAvoidance.set(true);
         }
      }
      else
      {
         timeRemainingToDisableCollapseAvoidance.set(timeDelayToDisableCollapseAvoidance.getDoubleValue());
         doSmoothTransitionOutOfCollapseAvoidance.set(false);
      }

      // Check if there is a risk that the support knee collapse
      if (desiredPercentOfLegLength.getDoubleValue() > percentOfLegLengthThresholdForCollapseAvoidance.getDoubleValue())
      {
         alphaCollapseAvoidance.set(0.0);
         doSmoothTransitionOutOfCollapseAvoidance.set(isSupportCollapseAvoidanceUsed.getBooleanValue());
         timeRemainingToDisableCollapseAvoidance.set(0.0);
         if (!isSupportCollapseAvoidanceUsed.getBooleanValue())
            return;
      }
      else if (!isSupportCollapseAvoidanceUsed.getBooleanValue())
      {
         isSupportCollapseAvoidanceUsed.set(true);
         doSmoothTransitionOutOfCollapseAvoidance.set(false);
         timeRemainingToDisableCollapseAvoidance.set(timeDelayToDisableCollapseAvoidance.getDoubleValue());
         timeSwitchCollapseAvoidance.set(yoTime.getDoubleValue());

         heightCorrectedFilteredForCollapseAvoidance.reset();
         heightVelocityCorrectedFilteredForCollapseAvoidance.reset();
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.reset();
         heightCorrectedFilteredForCollapseAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightVelocityCorrectedFilteredForCollapseAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());
      }

      if (doSmoothTransitionOutOfCollapseAvoidance.getBooleanValue())
      {
         heightCorrectedFilteredForCollapseAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightVelocityCorrectedFilteredForCollapseAvoidance.set(comHeightDataToCorrect.getComHeightVelocity());
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.set(comHeightDataToCorrect.getComHeightAcceleration());

         comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForCollapseAvoidance.getDoubleValue());
         comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForCollapseAvoidance.getDoubleValue());
         comHeightDataToCorrect.setComHeightAcceleration(heightAcceleretionCorrectedFilteredForCollapseAvoidance.getDoubleValue());

         if (Math.abs(desiredCenterOfMassHeightPoint.getZ() - heightCorrectedFilteredForCollapseAvoidance.getDoubleValue()) <= 5e-3)
         {
            alphaCollapseAvoidance.set(0.0);
            isSupportCollapseAvoidanceUsed.set(false);
            doSmoothTransitionOutOfCollapseAvoidance.set(false);
         }
         return;
      }

      anklePosition.setToZero(endEffectorFrame);
      anklePosition.changeFrame(worldFrame);
      yoCurrentFootPosition.set(anklePosition);
      anklePosition.changeFrame(virtualLegTangentialFrameHipCentered);
      currentLegLength.set(-anklePosition.getZ());
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      alphaCollapseAvoidance.set((desiredPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdForCollapseAvoidance.getDoubleValue())
            / (minPercentOfLegLengthForCollapseAvoidance.getDoubleValue() - percentOfLegLengthThresholdForCollapseAvoidance.getDoubleValue()));
      alphaCollapseAvoidance.set(MathTools.clamp(alphaCollapseAvoidance.getDoubleValue(), 0.0, 1.0));

      double desiredOrMinLegLength = Math.max(desiredLegLength.getDoubleValue(),
            minPercentOfLegLengthForCollapseAvoidance.getDoubleValue() * maximumLegLength.getDoubleValue());
      double correctedDesiredTranslationZ = desiredOrMinLegLength - currentLegLength.getDoubleValue(); //(1.0 - alphaCollapseAvoidance.getDoubleValue()) * equivalentDesiredHipPitchHeightTranslation.getZ() + alphaCollapseAvoidance.getDoubleValue() * (desiredOrMinLegLength - currentLegLength.getDoubleValue());
      equivalentDesiredHipPitchHeightTranslation.setZ(correctedDesiredTranslationZ);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(worldFrame);

      correctedDesiredLegLength.set(correctedDesiredTranslationZ + currentLegLength.getDoubleValue());
      correctedDesiredPercentOfLegLength.set(correctedDesiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      desiredCenterOfMassHeightPoint.setZ(zCurrent + equivalentDesiredHipPitchHeightTranslation.getZ());
      heightCorrectedFilteredForCollapseAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
      comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForCollapseAvoidance.getDoubleValue());

      if (equivalentDesiredHipVelocity.getZ() < 0.0) // Check if desired velocity results in knee flexion
      {
         equivalentDesiredHipVelocity.setZ((1.0 - alphaCollapseAvoidance.getDoubleValue()) * equivalentDesiredHipVelocity.getZ());
         equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
         if (Math.abs(comXYVelocity.getX()) > 1e-3 && Math.abs(equivalentDesiredHipVelocity.getX()) > 1e-3)
            equivalentDesiredHipVelocity.scale(comXYVelocity.getX() / equivalentDesiredHipVelocity.getX());
         equivalentDesiredHipVelocity.changeFrame(worldFrame);
         heightVelocityCorrectedFilteredForCollapseAvoidance.update(equivalentDesiredHipVelocity.getZ());
         comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForCollapseAvoidance.getDoubleValue());
      }
      else
      {
         heightVelocityCorrectedFilteredForCollapseAvoidance.reset();
         heightVelocityCorrectedFilteredForCollapseAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
      }

      if (equivalentDesiredHipPitchAcceleration.getZ() < 0.0) // Check if desired acceleration results in knee flexion
      {
         equivalentDesiredHipPitchAcceleration.setZ((1.0 - alphaCollapseAvoidance.getDoubleValue()) * equivalentDesiredHipPitchAcceleration.getZ());
         equivalentDesiredHipPitchAcceleration.changeFrame(worldFrame);
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.update(equivalentDesiredHipPitchAcceleration.getZ());
         comHeightDataToCorrect.setComHeightAcceleration(heightAcceleretionCorrectedFilteredForCollapseAvoidance.getDoubleValue());
      }
      else
      {
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.reset();
         heightAcceleretionCorrectedFilteredForCollapseAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());
      }
   }

   public void correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesData comHeightDataToCorrect, ConstraintType constraintType)
   {
      isUnreachableFootstepCompensated.set(false);

      if (!USE_UNREACHABLE_FOOTSTEP_CORRECTION)
         return;

      if (constraintType != ConstraintType.SWING && constraintType != ConstraintType.MOVE_VIA_WAYPOINTS)
         return;

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);

      if (unachievedSwingTranslation.getZ() < 0.0)
      {
         isUnreachableFootstepCompensated.set(true);
         unachievedSwingTranslationFiltered.update(unachievedSwingTranslation.getZ());
         desiredCenterOfMassHeightPoint.setZ(desiredCenterOfMassHeightPoint.getZ() + unachievedSwingTranslationFiltered.getDoubleValue());
         if (USE_UNREACHABLE_FOOTSTEP_CORRECTION_ON_POSITION)
            comHeightDataToCorrect.setComHeight(worldFrame, desiredCenterOfMassHeightPoint.getZ());
      }
      else
      {
         unachievedSwingTranslationFiltered.set(0.0);
      }

      if (unachievedSwingVelocity.getZ() < 0.0)
      {
         unachievedSwingVelocityFiltered.update(unachievedSwingVelocity.getZ());
         comHeightDataToCorrect.setComHeightVelocity(comHeightDataToCorrect.getComHeightVelocity() + unachievedSwingVelocityFiltered.getDoubleValue());
      }
      else
      {
         unachievedSwingVelocityFiltered.set(0.0);
      }

      if (unachievedSwingAcceleration.getZ() < 0.0)
      {
         unachievedSwingAccelerationFiltered.update(unachievedSwingAcceleration.getZ());
         comHeightDataToCorrect
               .setComHeightAcceleration(comHeightDataToCorrect.getComHeightAcceleration() + unachievedSwingAccelerationFiltered.getDoubleValue());
      }
      else
      {
         unachievedSwingAccelerationFiltered.set(0.0);
      }
   }
}
