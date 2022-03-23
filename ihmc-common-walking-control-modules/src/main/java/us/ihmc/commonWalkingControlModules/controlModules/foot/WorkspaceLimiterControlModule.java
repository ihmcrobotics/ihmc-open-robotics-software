package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesDataBasics;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.net.Vector3DSerializer;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WorkspaceLimiterControlModule
{
   private static final double epsilon = 5e-3;
   private boolean visualize = true;
   private boolean moreVisualizers = true;

   private static final boolean USE_UNREACHABLE_FOOTSTEP_CORRECTION = true; // Lower the CoM if a footstep is unreachable

   private final BooleanParameter useSingularityAvoidanceInSwing;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame virtualLegTangentialFrameHipCentered, virtualLegTangentialFrameAnkleCentered;

   private final YoRegistry registry;

   private final YoBoolean checkVelocityForSwingSingularityAvoidance;

   private final YoDouble alphaSwingSingularityAvoidanceForFoot;
   private final YoDouble alphaSwingSingularityAvoidanceForHeight;

   private final YoDouble maximumLegLength;

   private final DoubleProvider percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot;
   private final DoubleProvider percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight;
   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot;
   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight;
   private final DoubleProvider minPercentOfLegLengthForSingularityAvoidanceInSwing;

   private final DoubleProvider velocityDifferenceForLengthening;

   private final YoDouble desiredPercentOfLegLength;
   private final YoDouble currentPercentOfLegLength;

   private final YoDouble desiredLegLength;
   private final YoDouble currentLegLength;

   private final RigidBodyBasics pelvis;

   private final FrameVector3D tempVector = new FrameVector3D();
   private final YoFrameVector3D unachievedSwingTranslation;
   private final YoFrameVector3D unachievedSwingVelocity;
   private final YoFrameVector3D unachievedSwingAcceleration;

   private final FramePoint3D desiredCenterOfMassHeightPoint = new FramePoint3D(worldFrame);
   private final FramePoint3D anklePosition = new FramePoint3D(worldFrame);

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

   private final YoFrameVector3D yoDesiredFootLinearAcceleration;
   private final YoFrameVector3D yoCorrectedDesiredFootLinearAcceleration;

   private final YoGraphicReferenceFrame virtualLegTangentialFrameHipCenteredGraphics, virtualLegTangentialFrameAnkleCenteredGraphics;
   private final YoGraphicPosition yoDesiredFootPositionGraphic, yoCorrectedDesiredFootPositionGraphic;
   private final YoGraphicVector yoDesiredFootLinearVelocityGraphic, yoCorrectedDesiredFootLinearVelocityGraphic;

   private final YoBoolean isSwingSingularityAvoidanceUsed;
   private final YoBoolean isSwingSingularityAvoidanceUsedOnHeight;
   private final YoBoolean isUnreachableFootstepCompensated;

   private final DoubleProvider timeToCorrectForUnachievedSwingTranslation;
   private final AlphaFilteredYoVariable unachievedSwingTranslationFiltered;
   private final AlphaFilteredYoVariable unachievedSwingVelocityFiltered;
   private final AlphaFilteredYoVariable unachievedSwingAccelerationFiltered;

   private final YoBoolean doSmoothTransitionOutOfSingularityAvoidance;
   private final YoBoolean doSmoothTransitionOutOfUnreachableStep;

   private final YoDouble alphaSupportSingularityAvoidance;

   private final BooleanParameter useSingularityAvoidanceInSupport;

   private final YoBoolean isSupportSingularityAvoidanceUsed;
   private final DoubleProvider percentOfLegLengthMarginToDisableSingularityAvoidance;
   private final DoubleProvider percentOfLegLengthMarginToAbortSingularityAvoidance;

   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSupport;

   private final FrameVector3D equivalentDesiredHipPitchHeightTranslation = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipVelocity = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipPitchAcceleration = new FrameVector3D();

   private final AlphaFilteredYoVariable heightCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightVelocityCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightAccelerationCorrectedFilteredForSingularityAvoidance;

   public WorkspaceLimiterControlModule(String namePrefix,
                                        ContactablePlaneBody contactablePlaneBody,
                                        final RobotSide robotSide,
                                        WorkspaceLimiterParameters workspaceLimiterParameters,
                                        WalkingControllerParameters walkingControllerParameters,
                                        final HighLevelHumanoidControllerToolbox controllerToolbox,
                                        YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      unachievedSwingTranslation = new YoFrameVector3D(namePrefix + "UnachievedSwingTranslation", worldFrame, registry);
      unachievedSwingVelocity = new YoFrameVector3D(namePrefix + "UnachievedSwingVelocity", worldFrame, registry);
      unachievedSwingAcceleration = new YoFrameVector3D(namePrefix + "UnachievedSwingAcceleration", worldFrame, registry);

      maximumLegLength = new YoDouble(namePrefix + "MaxLegLength", registry);
      maximumLegLength.set(walkingControllerParameters.getMaximumLegLengthForSingularityAvoidance());

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      frameBeforeHipPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getFrameBeforeJoint();
      endEffectorFrame = contactablePlaneBody.getFrameAfterParentJoint();

      checkVelocityForSwingSingularityAvoidance = new YoBoolean(namePrefix + "CheckVelocityForSwingSingularityAvoidance", registry);

      alphaSwingSingularityAvoidanceForFoot = new YoDouble(namePrefix + "AlphaSwingSingularityAvoidanceForFoot", registry);
      alphaSwingSingularityAvoidanceForHeight = new YoDouble(namePrefix + "AlphaSwingSingularityAvoidanceForHeight", registry);
      DoubleProvider alphaUnreachableFootstep = workspaceLimiterParameters.getAlphaUnreachableFootstep();

      timeToCorrectForUnachievedSwingTranslation = workspaceLimiterParameters.getTimeToCorrectForUnachievedSwingTranslation();

      unachievedSwingTranslationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingTranslationFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingVelocityFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingVelocityFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingAccelerationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingAccelerationFiltered", registry, alphaUnreachableFootstep);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport = workspaceLimiterParameters.getMaxPercentOfLegLengthForSingularityAvoidanceInSupport();
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot = workspaceLimiterParameters.getMaxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot();
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight = workspaceLimiterParameters.getMaxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight();
      minPercentOfLegLengthForSingularityAvoidanceInSwing = workspaceLimiterParameters.getMinPercentOfLegLengthForSingularityAvoidanceInSwing();
      percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot = workspaceLimiterParameters.getPercentOfLegLengthMarginToEnableSingularityAvoidanceForFoot();
      percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight = workspaceLimiterParameters.getPercentOfLegLengthMarginToEnableSingularityAvoidanceForHeight();
      percentOfLegLengthMarginToDisableSingularityAvoidance = workspaceLimiterParameters.getPercentOfLegLengthMarginToDisableSingularityAvoidance();
      percentOfLegLengthMarginToAbortSingularityAvoidance = workspaceLimiterParameters.getPercentOfLegLengthMarginToAbortSingularityAvoidance();

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      useSingularityAvoidanceInSwing = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSwing",
                                                            registry,
                                                            swingTrajectoryParameters.useSingularityAvoidanceInSwing());

      desiredPercentOfLegLength = new YoDouble(namePrefix + "DesiredPercentOfLegLength", registry);
      currentPercentOfLegLength = new YoDouble(namePrefix + "CurrentPercentOfLegLength", registry);

      velocityDifferenceForLengthening = workspaceLimiterParameters.getVelocityDifferenceForLengthening();

      desiredLegLength = new YoDouble(namePrefix + "DesiredLegLength", registry);
      currentLegLength = new YoDouble(namePrefix + "CurrentLegLength", registry);

      isSwingSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSwingSingularityAvoidanceUsed", registry);
      isSwingSingularityAvoidanceUsedOnHeight = new YoBoolean(namePrefix + "IsSwingSingularityAvoidanceUsedOnHeight", registry);
      isUnreachableFootstepCompensated = new YoBoolean(namePrefix + "IsUnreachableFootstepCompensated", registry);

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
            EuclidGeometryTools.orientation3DFromZUpToVector3D(footToHipAxis, hipPitchRotationToParentFrame);
            hipPitchPosition.setToZero(frameBeforeHipPitchJoint);
            hipPitchPosition.changeFrame(getParent());

            transformToParent.getRotation().set(hipPitchRotationToParentFrame);
            transformToParent.getTranslation().set(hipPitchPosition);
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
            EuclidGeometryTools.orientation3DFromZUpToVector3D(footToHipAxis, anklePitchRotationToParentFrame);
            anklePitchPosition.setToZero(endEffectorFrame);
            anklePitchPosition.changeFrame(getParent());

            transformToParent.getRotation().set(anklePitchRotationToParentFrame);
            transformToParent.getTranslation().set(anklePitchPosition);
         }
      };

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      visualize = visualize && yoGraphicsListRegistry != null;
      moreVisualizers = visualize && moreVisualizers;

      yoCurrentFootPosition = new YoFramePoint3D(namePrefix + "CurrentFootPosition", worldFrame, registry);
      yoDesiredFootPosition = new YoFramePoint3D(namePrefix + "DesiredFootPosition", worldFrame, registry);
      yoCorrectedDesiredFootPosition = new YoFramePoint3D(namePrefix + "CorrectedDesiredFootPosition", worldFrame, registry);
      yoDesiredFootLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredFootLinearVelocity", worldFrame, registry);
      yoDesiredFootLinearAcceleration = new YoFrameVector3D(namePrefix + "DesiredFootLinearAcceleration", worldFrame, registry);
      yoCorrectedDesiredFootLinearVelocity = new YoFrameVector3D(namePrefix + "CorrectedDesiredFootLinearVelocity", worldFrame, registry);
      yoCorrectedDesiredFootLinearAcceleration = new YoFrameVector3D(namePrefix + "CorrectedDesiredFootLinearAcceleration", worldFrame, registry);
      yoDesiredFootPosition.setToNaN();
      yoCorrectedDesiredFootPosition.setToNaN();
      yoDesiredFootLinearVelocity.setToNaN();
      yoCorrectedDesiredFootLinearVelocity.setToNaN();

      alphaSupportSingularityAvoidance = new YoDouble(namePrefix + "AlphaSupportSingularityAvoidance", registry);

      doSmoothTransitionOutOfSingularityAvoidance = new YoBoolean(namePrefix + "DoSmoothTransitionSingularityAvoidance", registry);
      doSmoothTransitionOutOfUnreachableStep = new YoBoolean(namePrefix + "DoSmoothTransitionUnreachableStep", registry);

      isSupportSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSupportSingularityAvoidanceUsed", registry);
      useSingularityAvoidanceInSupport = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSupport",
                                                              registry,
                                                              swingTrajectoryParameters.useSingularityAvoidanceInSupport());

      DoubleProvider correctionAlphaFilter = workspaceLimiterParameters.getCorrectionAlphaFilter();
      heightCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(namePrefix + "HeightCorrectedFilteredForSingularityAvoidance",
                                                                                   registry,
                                                                                   correctionAlphaFilter);
      heightVelocityCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightVelocityCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);
      heightAccelerationCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightAccelerationCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);

      if (visualize)
      {
         yoDesiredFootPositionGraphic = new YoGraphicPosition(namePrefix + "DesiredFootPosition",
                                                              yoDesiredFootPosition,
                                                              0.025,
                                                              YoAppearance.Red(),
                                                              GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootPositionGraphic);
         yoCorrectedDesiredFootPositionGraphic = new YoGraphicPosition(namePrefix + "CorrectedDesiredFootPosition",
                                                                       yoCorrectedDesiredFootPosition,
                                                                       0.025,
                                                                       YoAppearance.Green(),
                                                                       GraphicType.BALL);
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

         yoDesiredFootLinearVelocityGraphic = new YoGraphicVector(namePrefix + "DesiredFootLinearVelocity",
                                                                  yoDesiredFootPosition,
                                                                  yoDesiredFootLinearVelocity,
                                                                  0.2,
                                                                  YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootLinearVelocityGraphic);
         yoCorrectedDesiredFootLinearVelocityGraphic = new YoGraphicVector(namePrefix + "CorrectedDesiredFootLinearVelocity",
                                                                           yoCorrectedDesiredFootPosition,
                                                                           yoCorrectedDesiredFootLinearVelocity,
                                                                           0.2,
                                                                           YoAppearance.Green());
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
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
   }

   public void resetHeightCorrectionParameters()
   {
      doSmoothTransitionOutOfSingularityAvoidance.set(false);

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

      alphaSwingSingularityAvoidanceForFoot.set(0.0);
      alphaSwingSingularityAvoidanceForHeight.set(0.0);
      unachievedSwingTranslation.setToZero();
      unachievedSwingVelocity.setToZero();
      unachievedSwingAcceleration.setToZero();
   }

   public void setCheckVelocityForSwingSingularityAvoidance(boolean value)
   {
      checkVelocityForSwingSingularityAvoidance.set(value);
   }

   public void correctSwingFootTrajectory(FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                          FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                          FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
   {
      isSwingSingularityAvoidanceUsed.set(false);
      isSwingSingularityAvoidanceUsedOnHeight.set(false);
      alphaSwingSingularityAvoidanceForFoot.set(0.0);
      alphaSwingSingularityAvoidanceForHeight.set(0.0);

      yoDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);
      yoDesiredFootLinearAcceleration.set(desiredFootLinearAccelerationToCorrect);

      desiredFootPosition.setIncludingFrame(desiredFootPositionToCorrect);
      desiredFootLinearVelocity.setIncludingFrame(desiredFootLinearVelocityToCorrect);
      desiredFootLinearAcceleration.setIncludingFrame(desiredFootLinearAccelerationToCorrect);

      desiredFootPosition.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
      desiredFootLinearAcceleration.changeFrame(virtualLegTangentialFrameAnkleCentered);

      desiredLegLength.set(-desiredFootPosition.getZ());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      if (useSingularityAvoidanceInSwing.getValue())
      {
         desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
         pelvis.getBodyFixedFrame().getTwistOfFrame(pelvisTwist);
         pelvisLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
         pelvisLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

         double upperBoundToStartHeightCorrection =
               maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight.getValue() - percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight.getValue();

         if (desiredPercentOfLegLength.getDoubleValue() > upperBoundToStartHeightCorrection)
         {
            correctHeightForOverExtensionForSingularityAvoidance();
         }

         double upperBoundToStartFootCorrection =
               maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot.getValue() - percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot.getValue();
         double lowerBoundToStartFootCorrection =
               minPercentOfLegLengthForSingularityAvoidanceInSwing.getValue() + percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot.getValue();

         if (desiredPercentOfLegLength.getDoubleValue() > upperBoundToStartFootCorrection)
         {
            correctSwingFootTrajectoryOverExtensionForSingularityAvoidance(desiredFootPositionToCorrect,
                                                                           desiredFootLinearVelocityToCorrect,
                                                                           desiredFootLinearAccelerationToCorrect);
         }
         else if (desiredPercentOfLegLength.getDoubleValue() < lowerBoundToStartFootCorrection)
         {
            correctSwingFootTrajectoryUnderExtensionForSingularityAvoidance(desiredFootPositionToCorrect,
                                                                            desiredFootLinearVelocityToCorrect,
                                                                            desiredFootLinearAccelerationToCorrect);
         }
      }
   }

   private void correctHeightForOverExtensionForSingularityAvoidance()
   {
      // foot's being picked up more quickly than the pelvis is.
      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && isLegShortening())
         return;

      isSwingSingularityAvoidanceUsedOnHeight.set(true);

      updateFractionOfSingularityAvoidanceToUse(alphaSwingSingularityAvoidanceForHeight,
                                                maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight.getValue(),
                                                percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight.getValue());

      double desiredFootPositionInAxisFrame = -Math.min(desiredLegLength.getDoubleValue(),
                                                        maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight.getValue()
                                                        * maximumLegLength.getDoubleValue());

      // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
      double desiredLinearVelocityZ = InterpolationTools.linearInterpolate(desiredFootLinearVelocity.getZ(),
                                                                           pelvisLinearVelocity.getZ(),
                                                                           alphaSwingSingularityAvoidanceForHeight.getDoubleValue());

      double unachievedHeightTranslation = desiredFootPosition.getZ() - desiredFootPositionInAxisFrame;
      double unachievedHeightVelocity = desiredFootLinearVelocity.getZ() - desiredLinearVelocityZ;
      double unachievedHeightAcceleration = alphaSwingSingularityAvoidanceForHeight.getDoubleValue() * desiredFootLinearAcceleration.getZ();

      tempVector.setIncludingFrame(desiredFootPosition.getReferenceFrame(), 0.0, 0.0, unachievedHeightTranslation);
      unachievedSwingTranslation.setMatchingFrame(tempVector);

      tempVector.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0, unachievedHeightVelocity);
      unachievedSwingVelocity.setMatchingFrame(tempVector);

      tempVector.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(),
                                      0.0,
                                      0.0,
                                      unachievedHeightAcceleration);
      unachievedSwingAcceleration.setMatchingFrame(tempVector);
   }

   private void correctSwingFootTrajectoryOverExtensionForSingularityAvoidance(FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                                                               FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                                                               FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
   {

      // foot's being picked up more quickly than the pelvis is.
      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && isLegShortening())
         return;

      checkVelocityForSwingSingularityAvoidance.set(false);
      isSwingSingularityAvoidanceUsed.set(true);

      updateFractionOfSingularityAvoidanceToUse(alphaSwingSingularityAvoidanceForFoot,
                                                maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot.getValue(),
                                                percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot.getValue());

      double desiredFootPositionInAxisFrame = -Math.min(desiredLegLength.getDoubleValue(),
                                                        maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot.getValue()
                                                        * maximumLegLength.getDoubleValue());

      correctFootDesiredsWithScaleFactor(desiredFootPositionInAxisFrame,
                                         desiredFootPositionToCorrect,
                                         desiredFootLinearVelocityToCorrect,
                                         desiredFootLinearAccelerationToCorrect);
   }

   private void correctSwingFootTrajectoryUnderExtensionForSingularityAvoidance(FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                                                                FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                                                                FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
   {
      // foot's being picked up less quickly than the pelvis is.
      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && isLegLengthening())
         return;

      checkVelocityForSwingSingularityAvoidance.set(false);
      isSwingSingularityAvoidanceUsed.set(true);

      double alpha = (desiredPercentOfLegLength.getDoubleValue() - minPercentOfLegLengthForSingularityAvoidanceInSwing.getValue())
                     / percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot.getValue();
      alphaSwingSingularityAvoidanceForFoot.set(1.0 - MathTools.clamp(alpha, 0.0, 1.0));

      double desiredFootPositionInAxisFrame = -Math.max(desiredLegLength.getDoubleValue(),
                                                        minPercentOfLegLengthForSingularityAvoidanceInSwing.getValue()
                                                        * maximumLegLength.getDoubleValue());

      correctFootDesiredsWithScaleFactor(desiredFootPositionInAxisFrame,
                                         desiredFootPositionToCorrect,
                                         desiredFootLinearVelocityToCorrect,
                                         desiredFootLinearAccelerationToCorrect);
   }

   private void correctFootDesiredsWithScaleFactor(double desiredFootPositionInAxisFrame,
                                                   FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                                   FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                                   FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
   {
      desiredFootLinearVelocity.checkReferenceFrameMatch(virtualLegTangentialFrameAnkleCentered);
      desiredFootLinearAcceleration.checkReferenceFrameMatch(virtualLegTangentialFrameAnkleCentered);
      pelvisLinearVelocity.checkReferenceFrameMatch(virtualLegTangentialFrameAnkleCentered);

      // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
      double desiredLinearVelocityZ = desiredFootLinearVelocity.getZ();
      double desiredLinearAccelerationZ = desiredFootLinearAcceleration.getZ();
      if (isLegLengthening())
      {
         desiredLinearVelocityZ = InterpolationTools.linearInterpolate(desiredFootLinearVelocity.getZ(),
                                              pelvisLinearVelocity.getZ(),
                                              alphaSwingSingularityAvoidanceForFoot.getDoubleValue());
         desiredLinearAccelerationZ = InterpolationTools.linearInterpolate(desiredFootLinearAcceleration.getZ(),
                                                                           0.0,
                                                                           alphaSwingSingularityAvoidanceForFoot.getDoubleValue());
      }
//      desiredFootLinearAcceleration.interpolate(EuclidCoreTools.zeroVector3D, alphaSwingSingularityAvoidanceForFoot.getDoubleValue());


      desiredFootPosition.setZ(desiredFootPositionInAxisFrame);
      desiredFootLinearVelocity.setZ(desiredLinearVelocityZ);
      desiredFootLinearAcceleration.setZ(desiredLinearAccelerationZ);


      desiredFootPositionToCorrect.setMatchingFrame(desiredFootPosition);
      desiredFootLinearVelocityToCorrect.setMatchingFrame(desiredFootLinearVelocity);
      desiredFootLinearAccelerationToCorrect.setMatchingFrame(desiredFootLinearAcceleration);

      yoCorrectedDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoCorrectedDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);
      yoCorrectedDesiredFootLinearAcceleration.set(desiredFootLinearAccelerationToCorrect);

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

   private boolean isLegLengthening()
   {
      pelvisLinearVelocity.checkReferenceFrameMatch(desiredFootLinearVelocity);
      return pelvisLinearVelocity.getZ() - desiredFootLinearVelocity.getZ() > velocityDifferenceForLengthening.getValue();
   }

   private boolean isLegShortening()
   {
      pelvisLinearVelocity.checkReferenceFrameMatch(desiredFootLinearVelocity);
      return desiredFootLinearVelocity.getZ() - pelvisLinearVelocity.getZ() > velocityDifferenceForLengthening.getValue();
   }

   public boolean correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect, ConstraintType constraintType)
   {
      isUnreachableFootstepCompensated.set(false);

      if (!USE_UNREACHABLE_FOOTSTEP_CORRECTION)
         return false;

      if (constraintType.isLoadBearing())
      {
         return smoothTransitionOutOfHeightCorrectionInSwing(comHeightDataToCorrect, constraintType);
//         return false;
      }
      else
      {
         doSmoothTransitionOutOfUnreachableStep.set(false);
      }

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);

      if (unachievedSwingTranslation.getZ() < 0.0)
      {
         isUnreachableFootstepCompensated.set(true);
         unachievedSwingTranslationFiltered.update(unachievedSwingTranslation.getZ());
         desiredCenterOfMassHeightPoint.addZ(unachievedSwingTranslationFiltered.getDoubleValue());

         comHeightDataToCorrect.setComHeight(worldFrame, desiredCenterOfMassHeightPoint.getZ());

         unachievedSwingVelocityFiltered.update(
               unachievedSwingTranslationFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getValue());
         unachievedSwingAccelerationFiltered.update(
               unachievedSwingVelocityFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getValue());

         comHeightDataToCorrect.setComHeightVelocity(comHeightDataToCorrect.getComHeightVelocity() + unachievedSwingVelocityFiltered.getDoubleValue());
         comHeightDataToCorrect.setComHeightAcceleration(
                  comHeightDataToCorrect.getComHeightAcceleration() + unachievedSwingAccelerationFiltered.getDoubleValue());

         return true;
      }
      else
      {
         unachievedSwingTranslationFiltered.set(0.0);
         unachievedSwingVelocityFiltered.set(0.0);
         unachievedSwingAccelerationFiltered.set(0.0);

         return false;
      }
   }

   public boolean correctCoMHeightTrajectoryForSingularityAvoidanceInSupport(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect,
                                                                             double zCurrentInWorld,
                                                                             ReferenceFrame pelvisZUpFrame,
                                                                             ConstraintType constraintType)
   {
      if (!useSingularityAvoidanceInSupport.getValue())
      {
         alphaSupportSingularityAvoidance.set(0.0);
         isSupportSingularityAvoidanceUsed.set(false);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);
         return false;
      }

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);
      double heightErrorInWorld = desiredCenterOfMassHeightPoint.getZ() - zCurrentInWorld;
      equivalentDesiredHipPitchHeightTranslation.setIncludingFrame(worldFrame, 0.0, 0.0, heightErrorInWorld);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(virtualLegTangentialFrameAnkleCentered);

      desiredLegLength.set(equivalentDesiredHipPitchHeightTranslation.getZ() + currentLegLength.getDoubleValue());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      // we're in toe off or swing, so don't do anything, but indicate that whether or not to smooth next time
      if (constraintType != ConstraintType.FULL)
      {
         alphaSupportSingularityAvoidance.set(0.0);
         doSmoothTransitionOutOfSingularityAvoidance.set(isSupportSingularityAvoidanceUsed.getBooleanValue());
         if (!isSupportSingularityAvoidanceUsed.getBooleanValue())
            return false;
      }

      double maxPercent = maxPercentOfLegLengthForSingularityAvoidanceInSupport.getValue();
      boolean singularityAvoidanceShouldBeDisabled =
            desiredPercentOfLegLength.getDoubleValue() < maxPercent - percentOfLegLengthMarginToDisableSingularityAvoidance.getValue();
      boolean legDoesNotNeedSingularityAvoidance =
            desiredPercentOfLegLength.getDoubleValue() < maxPercent - percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight.getValue();

      // This checks to see if we were doing singularity avoidance, but the leg is straight now, and we aren't already transitioning out of using
      // singularity avoidance.
      if (isSupportSingularityAvoidanceUsed.getBooleanValue() && singularityAvoidanceShouldBeDisabled
          && !doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
      {
         alphaSupportSingularityAvoidance.set(0.0);
         doSmoothTransitionOutOfSingularityAvoidance.set(true);
      }

      // Check if the leg is extended such as the trajectory needs to be corrected
      if (legDoesNotNeedSingularityAvoidance)
      {
         // if we haven't been using singularity avoidance, and we aren't currently transitioning out of it, then the current desired values are good.
         if (!isSupportSingularityAvoidanceUsed.getBooleanValue() && !doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
            return false;
      }
      else if (!isSupportSingularityAvoidanceUsed.getBooleanValue())
      {  // The leg is too straight, so we need to use singularity avoidance, but we haven't set it up yet, so set up singularity avoidance and start it.
         isSupportSingularityAvoidanceUsed.set(true);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);

         heightCorrectedFilteredForSingularityAvoidance.reset();
         heightVelocityCorrectedFilteredForSingularityAvoidance.reset();
         heightAccelerationCorrectedFilteredForSingularityAvoidance.reset();
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightVelocityCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
         heightAccelerationCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());
      }

      if (doSmoothTransitionOutOfSingularityAvoidance.getBooleanValue())
      {
         smoothTransitionOutOfSingularityAvoidanceInSupport(comHeightDataToCorrect);
      }
      else
      {
         applySingularityAvoidanceInSupport(comHeightDataToCorrect, zCurrentInWorld, pelvisZUpFrame);
      }

      return true;
   }

   private void updateFractionOfSingularityAvoidanceToUse(YoDouble alphaToUpdate, double percentForMaximum, double percentMargin)
   {
      double percentToStart = percentForMaximum - percentMargin;
      double alpha = (desiredPercentOfLegLength.getDoubleValue() - percentToStart) / percentMargin;
      alphaToUpdate.set(MathTools.clamp(alpha, 0.0, 1.0));
   }

   private void smoothTransitionOutOfSingularityAvoidanceInSupport(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect)
   {
      heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
      heightVelocityCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
      heightAccelerationCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());

      comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      comHeightDataToCorrect.setComHeightAcceleration(heightAccelerationCorrectedFilteredForSingularityAvoidance.getDoubleValue());

      // If the filtered desired com height caught up to the input one, then stop doing smooth transition.
      if (MathTools.epsilonEquals(desiredCenterOfMassHeightPoint.getZ(), heightCorrectedFilteredForSingularityAvoidance.getDoubleValue(), epsilon))
      {
         alphaSupportSingularityAvoidance.set(0.0);
         isSupportSingularityAvoidanceUsed.set(false);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);
      }

      // If height is lower than filtered and the knee is bent enough, then really want to get out of singularity avoidance faster. So in this case, smooth faster...
      else if (desiredCenterOfMassHeightPoint.getZ() <= heightCorrectedFilteredForSingularityAvoidance.getDoubleValue() && (
            desiredPercentOfLegLength.getDoubleValue()
            < maxPercentOfLegLengthForSingularityAvoidanceInSupport.getValue() - percentOfLegLengthMarginToDisableSingularityAvoidance.getValue()))
      {
         // Call this twice here to smooth faster. Need to get out of singularity avoidance!
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());

         // If leg is bent a lot and singularity avoidance no longer needed, stop smoothing...
         if (desiredPercentOfLegLength.getDoubleValue()
             < maxPercentOfLegLengthForSingularityAvoidanceInSupport.getValue() - percentOfLegLengthMarginToAbortSingularityAvoidance.getValue())
         {
            alphaSupportSingularityAvoidance.set(0.0);
            isSupportSingularityAvoidanceUsed.set(false);
            doSmoothTransitionOutOfSingularityAvoidance.set(false);
         }
      }
   }

   private final FrameVector2DBasics comVelocity = new FrameVector2D();

   private void applySingularityAvoidanceInSupport(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect,
                                                   double zCurrent,
                                                   ReferenceFrame pelvisZUpFrame)
   {
      updateFractionOfSingularityAvoidanceToUse(alphaSupportSingularityAvoidance,
                                                maxPercentOfLegLengthForSingularityAvoidanceInSupport.getValue(),
                                                percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight.getValue());

      double desiredOrMaxLegLength = Math.min(desiredLegLength.getDoubleValue(),
                                              maxPercentOfLegLengthForSingularityAvoidanceInSupport.getValue() * maximumLegLength.getDoubleValue());
      double correctedDesiredTranslationZ = desiredOrMaxLegLength - currentLegLength.getDoubleValue();
      equivalentDesiredHipPitchHeightTranslation.setZ(correctedDesiredTranslationZ);
      equivalentDesiredHipPitchHeightTranslation.changeFrame(worldFrame);

      desiredCenterOfMassHeightPoint.setZ(zCurrent + equivalentDesiredHipPitchHeightTranslation.getZ());
      heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
      comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForSingularityAvoidance.getDoubleValue());

      // Correct the height velocity
      equivalentDesiredHipVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightVelocity());
      equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
      comVelocity.setIncludingFrame(comVelocity);
      comVelocity.changeFrame(pelvisZUpFrame);
      equivalentDesiredHipVelocity.setX(comVelocity.getX());
      equivalentDesiredHipVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      double scaledHipVelocity = InterpolationTools.linearInterpolate(equivalentDesiredHipVelocity.getZ(),
                                                                      0.0,
                                                                      alphaSupportSingularityAvoidance.getDoubleValue());
      equivalentDesiredHipVelocity.setZ(scaledHipVelocity);
      equivalentDesiredHipVelocity.changeFrame(pelvisZUpFrame);
      if (Math.abs(comVelocity.getX()) > 1e-3 && Math.abs(equivalentDesiredHipVelocity.getX()) > 1e-3)
         equivalentDesiredHipVelocity.scale(comVelocity.getX() / equivalentDesiredHipVelocity.getX());
      equivalentDesiredHipVelocity.changeFrame(worldFrame);
      heightVelocityCorrectedFilteredForSingularityAvoidance.update(equivalentDesiredHipVelocity.getZ());
      comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForSingularityAvoidance.getDoubleValue());

      // Correct the height acceleration
      equivalentDesiredHipPitchAcceleration.setIncludingFrame(worldFrame, 0.0, 0.0, comHeightDataToCorrect.getComHeightAcceleration());
      equivalentDesiredHipPitchAcceleration.changeFrame(virtualLegTangentialFrameAnkleCentered);

      double scaledHipAcceleration = InterpolationTools.linearInterpolate(equivalentDesiredHipPitchAcceleration.getZ(),
                                                                          0.0,
                                                                          alphaSupportSingularityAvoidance.getDoubleValue());
      equivalentDesiredHipPitchAcceleration.setZ(scaledHipAcceleration);
      equivalentDesiredHipPitchAcceleration.changeFrame(worldFrame);
      heightAccelerationCorrectedFilteredForSingularityAvoidance.update(equivalentDesiredHipPitchAcceleration.getZ());
      comHeightDataToCorrect.setComHeightAcceleration(heightAccelerationCorrectedFilteredForSingularityAvoidance.getDoubleValue());
   }

   private boolean smoothTransitionOutOfHeightCorrectionInSwing(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect, ConstraintType constraintType)
   {
      if (!USE_UNREACHABLE_FOOTSTEP_CORRECTION)
         return false;

      if (constraintType != ConstraintType.FULL)
         return false;

      unachievedSwingTranslationFiltered.update(0.0);

      unachievedSwingVelocityFiltered.update(unachievedSwingTranslationFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getValue());
      unachievedSwingAccelerationFiltered.update(
            unachievedSwingVelocityFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getValue());

      if (MathTools.epsilonEquals(unachievedSwingVelocityFiltered.getDoubleValue(), 0.0, epsilon))
      {
         doSmoothTransitionOutOfUnreachableStep.set(false);
         return false;
      }
      else
      {
         doSmoothTransitionOutOfUnreachableStep.set(true);
      }

      comHeightDataToCorrect.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCenterOfMassHeightPoint.changeFrame(worldFrame);

      desiredCenterOfMassHeightPoint.addZ(unachievedSwingTranslationFiltered.getDoubleValue());

      comHeightDataToCorrect.setComHeight(worldFrame, desiredCenterOfMassHeightPoint.getZ());

      comHeightDataToCorrect.setComHeightVelocity(comHeightDataToCorrect.getComHeightVelocity() + unachievedSwingVelocityFiltered.getDoubleValue());
      comHeightDataToCorrect.setComHeightAcceleration(comHeightDataToCorrect.getComHeightAcceleration() + unachievedSwingAccelerationFiltered.getDoubleValue());

      return true;
   }
}
