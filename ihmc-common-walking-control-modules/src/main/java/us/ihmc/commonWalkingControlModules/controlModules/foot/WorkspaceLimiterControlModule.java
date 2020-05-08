package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class WorkspaceLimiterControlModule
{
   private boolean visualize = true;
   private boolean moreVisualizers = true;

   private static final boolean USE_UNREACHABLE_FOOTSTEP_CORRECTION = true; // Lower the CoM if a footstep is unreachable
   private static final boolean USE_UNREACHABLE_FOOTSTEP_CORRECTION_ON_POSITION = true; // When false, the module will correct only the velocity and acceleration of the CoM height.

   private final BooleanParameter useSingularityAvoidanceInSwing;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame virtualLegTangentialFrameHipCentered, virtualLegTangentialFrameAnkleCentered;

   private final YoVariableRegistry registry;

   private final YoBoolean checkVelocityForSwingSingularityAvoidance;

   private final YoDouble alphaSwingSingularityAvoidance;
   private final YoDouble alphaUnreachableFootstep;

   private final YoDouble maximumLegLength;

   private final YoDouble percentOfLegLengthMarginToEnableSingularityAvoidance;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSwing;
   private final YoDouble minPercentOfLegLengthForSingularityAvoidanceInSwing;

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
   private final YoBoolean isUnreachableFootstepCompensated;

   private final YoDouble timeToCorrectForUnachievedSwingTranslation;
   private final AlphaFilteredYoVariable unachievedSwingTranslationFiltered;
   private final AlphaFilteredYoVariable unachievedSwingVelocityFiltered;
   private final AlphaFilteredYoVariable unachievedSwingAccelerationFiltered;

   private final YoBoolean doSmoothTransitionOutOfSingularityAvoidance;

   private final YoDouble alphaSupportSingularityAvoidance;

   private final BooleanParameter useSingularityAvoidanceInSupport;

   private final YoBoolean isSupportSingularityAvoidanceUsed;
   private final YoDouble percentOfLegLengthMarginToDisableSingularityAvoidance;
   private final YoDouble percentOfLegLengthMarginToAbortSingularityAvoidance;

   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSupport;

   private final FrameVector3D equivalentDesiredHipPitchHeightTranslation = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipVelocity = new FrameVector3D();
   private final FrameVector3D equivalentDesiredHipPitchAcceleration = new FrameVector3D();

   private final YoDouble correctionAlphaFilter;
   private final AlphaFilteredYoVariable heightCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightVelocityCorrectedFilteredForSingularityAvoidance;
   private final AlphaFilteredYoVariable heightAccelerationCorrectedFilteredForSingularityAvoidance;

   public WorkspaceLimiterControlModule(String namePrefix,
                                        ContactablePlaneBody contactablePlaneBody,
                                        final RobotSide robotSide,
                                        WalkingControllerParameters walkingControllerParameters,
                                        final HighLevelHumanoidControllerToolbox controllerToolbox,
                                        YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
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

      alphaSwingSingularityAvoidance = new YoDouble(namePrefix + "AlphaSwingSingularityAvoidance", registry);
      alphaUnreachableFootstep = new YoDouble(namePrefix + "AlphaUnreachableFootstep", registry);
      alphaUnreachableFootstep.set(0.25);

      timeToCorrectForUnachievedSwingTranslation = new YoDouble(namePrefix + "TimeToCorrectForUnachievedSwingTranslation", registry);
      timeToCorrectForUnachievedSwingTranslation.set(0.2);

      unachievedSwingTranslationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingTranslationFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingVelocityFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingVelocityFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingAccelerationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingAccelerationFiltered", registry, alphaUnreachableFootstep);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSupport", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwing", registry);
      minPercentOfLegLengthForSingularityAvoidanceInSwing = new YoDouble(namePrefix + "MinPercOfLegLengthForSingularityAvoidanceInSwing", registry);
      percentOfLegLengthMarginToEnableSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToEnableSingularityAvoidance", registry);
      percentOfLegLengthMarginToDisableSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToDisableSingularityAvoidance", registry);
      percentOfLegLengthMarginToAbortSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToAbortSingularityAvoidance", registry);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport.set(0.98);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing.set(0.97);
      minPercentOfLegLengthForSingularityAvoidanceInSwing.set(0.5);

      percentOfLegLengthMarginToEnableSingularityAvoidance.set(0.1);
      percentOfLegLengthMarginToDisableSingularityAvoidance.set(0.12);
      percentOfLegLengthMarginToAbortSingularityAvoidance.set(0.17);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      useSingularityAvoidanceInSwing = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSwing",
                                                            registry,
                                                            swingTrajectoryParameters.useSingularityAvoidanceInSwing());

      desiredPercentOfLegLength = new YoDouble(namePrefix + "DesiredPercentOfLegLength", registry);
      currentPercentOfLegLength = new YoDouble(namePrefix + "CurrentPercentOfLegLength", registry);

      desiredLegLength = new YoDouble(namePrefix + "DesiredLegLength", registry);
      currentLegLength = new YoDouble(namePrefix + "CurrentLegLength", registry);

      isSwingSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSwingSingularityAvoidanceUsed", registry);
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

      isSupportSingularityAvoidanceUsed = new YoBoolean(namePrefix + "IsSupportSingularityAvoidanceUsed", registry);
      useSingularityAvoidanceInSupport = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSupport",
                                                              registry,
                                                              swingTrajectoryParameters.useSingularityAvoidanceInSupport());

      correctionAlphaFilter = new YoDouble(namePrefix + "CorrectionAlphaFilter", registry);
      heightCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(namePrefix + "HeightCorrectedFilteredForSingularityAvoidance",
                                                                                   registry,
                                                                                   correctionAlphaFilter);
      heightVelocityCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightVelocityCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);
      heightAccelerationCorrectedFilteredForSingularityAvoidance = new AlphaFilteredYoVariable(
            namePrefix + "HeightAccelerationCorrectedFilteredForSingularityAvoidance", registry, correctionAlphaFilter);
      correctionAlphaFilter.set(0.98);

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
                                                                  yoCurrentFootPosition,
                                                                  yoDesiredFootLinearVelocity,
                                                                  0.2,
                                                                  YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootLinearVelocityGraphic);
         yoCorrectedDesiredFootLinearVelocityGraphic = new YoGraphicVector(namePrefix + "CorrectedDesiredFootLinearVelocity",
                                                                           yoCurrentFootPosition,
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

      alphaSwingSingularityAvoidance.set(0.0);
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
      alphaSwingSingularityAvoidance.set(0.0);

      yoDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);
      yoDesiredFootLinearAcceleration.set(desiredFootLinearAccelerationToCorrect);

      desiredFootPosition.setIncludingFrame(desiredFootPositionToCorrect);
      desiredFootLinearVelocity.setIncludingFrame(desiredFootLinearVelocityToCorrect);
      desiredFootLinearAcceleration.setIncludingFrame(desiredFootLinearAccelerationToCorrect);

      desiredFootPosition.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearAcceleration.changeFrame(virtualLegTangentialFrameHipCentered);

      desiredLegLength.set(-desiredFootPosition.getZ());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());
      currentPercentOfLegLength.set(currentLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      if (useSingularityAvoidanceInSwing.getValue())
      {
         double upperBoundToStartSingularityAvoidance =
               maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() - percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();
         double lowerBoundToStartSingularityAvoidance =
               minPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() + percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();
         if (desiredPercentOfLegLength.getDoubleValue() > upperBoundToStartSingularityAvoidance)
         {
            correctSwingFootTrajectoryOverExtensionForSingularityAvoidance(desiredFootPositionToCorrect,
                                                                           desiredFootLinearVelocityToCorrect,
                                                                           desiredFootLinearAccelerationToCorrect);
         }
         else if (desiredPercentOfLegLength.getDoubleValue() < lowerBoundToStartSingularityAvoidance)
         {
            correctSwingFootTrajectoryUnderExtensionForSingularityAvoidance(desiredFootPositionToCorrect,
                                                                            desiredFootLinearVelocityToCorrect,
                                                                            desiredFootLinearAccelerationToCorrect);
         }
      }
   }

   private void correctSwingFootTrajectoryOverExtensionForSingularityAvoidance(FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                                                               FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                                                               FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
   {
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
      pelvis.getBodyFixedFrame().getTwistOfFrame(pelvisTwist);
      pelvisLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      pelvisLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      // foot's being picked up more quickly than the pelvis is.
      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && (desiredFootLinearVelocity.getZ() - pelvisLinearVelocity.getZ() > -1e-10))
         return;

      checkVelocityForSwingSingularityAvoidance.set(false);
      isSwingSingularityAvoidanceUsed.set(true);

      updateFractionOfSingularityAvoidanceToUse(alphaSwingSingularityAvoidance, maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue());

      double desiredFootPositionInAxisFrame = -Math.min(desiredLegLength.getDoubleValue(),
                                                        maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue()
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
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);
      pelvis.getBodyFixedFrame().getTwistOfFrame(pelvisTwist);
      pelvisLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      pelvisLinearVelocity.changeFrame(virtualLegTangentialFrameAnkleCentered);

      // foot's being picked up less quickly than the pelvis is.
      if (checkVelocityForSwingSingularityAvoidance.getBooleanValue() && (pelvisLinearVelocity.getZ() - desiredFootLinearVelocity.getZ()) > -1e-10)
         return;

      checkVelocityForSwingSingularityAvoidance.set(false);
      isSwingSingularityAvoidanceUsed.set(true);

      double alpha = (desiredPercentOfLegLength.getDoubleValue() - minPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue())
                     / percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();
      alphaSwingSingularityAvoidance.set(1.0 - MathTools.clamp(alpha, 0.0, 1.0));

      double desiredFootPositionInAxisFrame = -Math.max(desiredLegLength.getDoubleValue(),
                                                        minPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue()
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
      double desiredLinearVelocityX = desiredFootLinearVelocity.getX();
      double desiredLinearVelocityY = desiredFootLinearVelocity.getY();
      // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
      double desiredLinearVelocityZ = InterpolationTools.linearInterpolate(desiredFootLinearVelocity.getZ(),
                                                                           pelvisLinearVelocity.getZ(),
                                                                           alphaSwingSingularityAvoidance.getDoubleValue());
      double desiredLinearAccelerationZ = InterpolationTools.linearInterpolate(desiredFootLinearAcceleration.getZ(),
                                                                               0.0,
                                                                               alphaSwingSingularityAvoidance.getDoubleValue());

      tempVector.setIncludingFrame(desiredFootPosition.getReferenceFrame(), 0.0, 0.0, desiredFootPosition.getZ() - desiredFootPositionInAxisFrame);
      unachievedSwingTranslation.setMatchingFrame(tempVector);

      desiredFootPosition.setZ(desiredFootPositionInAxisFrame);

      tempVector.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(), 0.0, 0.0, desiredFootLinearVelocity.getZ() - desiredLinearVelocityZ);
      unachievedSwingVelocity.setMatchingFrame(tempVector);

      desiredFootLinearVelocity.setIncludingFrame(virtualLegTangentialFrameAnkleCentered,
                                                  desiredLinearVelocityX,
                                                  desiredLinearVelocityY,
                                                  desiredLinearVelocityZ);

      tempVector.setIncludingFrame(desiredFootLinearVelocity.getReferenceFrame(),
                                   0.0,
                                   0.0,
                                   alphaSwingSingularityAvoidance.getDoubleValue() * desiredFootLinearAcceleration.getZ());
      unachievedSwingAcceleration.setMatchingFrame(tempVector);

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
         desiredCenterOfMassHeightPoint.addZ(unachievedSwingTranslationFiltered.getDoubleValue());

         if (USE_UNREACHABLE_FOOTSTEP_CORRECTION_ON_POSITION)
         {
            comHeightDataToCorrect.setComHeight(worldFrame, desiredCenterOfMassHeightPoint.getZ());

            unachievedSwingVelocityFiltered.update(
                  unachievedSwingTranslationFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getDoubleValue());
            unachievedSwingAccelerationFiltered.update(
                  unachievedSwingVelocityFiltered.getDoubleValue() / timeToCorrectForUnachievedSwingTranslation.getDoubleValue());

            comHeightDataToCorrect.setComHeightVelocity(comHeightDataToCorrect.getComHeightVelocity() + unachievedSwingVelocityFiltered.getDoubleValue());
            comHeightDataToCorrect.setComHeightAcceleration(
                  comHeightDataToCorrect.getComHeightAcceleration() + unachievedSwingAccelerationFiltered.getDoubleValue());
         }
      }
      else
      {
         unachievedSwingTranslationFiltered.set(0.0);
         unachievedSwingVelocityFiltered.set(0.0);
         unachievedSwingAccelerationFiltered.set(0.0);
      }
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidanceInSupport(FrameVector2DReadOnly comXYVelocity,
                                                                          CoMHeightTimeDerivativesData comHeightDataToCorrect,
                                                                          double zCurrentInWorld,
                                                                          ReferenceFrame pelvisZUpFrame,
                                                                          ConstraintType constraintType)
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
            return;
      }

      double maxPercent = maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue();
      boolean singularityAvoidanceShouldBeDisabled =
            desiredPercentOfLegLength.getDoubleValue() < maxPercent - percentOfLegLengthMarginToDisableSingularityAvoidance.getDoubleValue();
      boolean legDoesNotNeedSingularityAvoidance =
            desiredPercentOfLegLength.getDoubleValue() < maxPercent - percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();

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
            return;
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
         applySingularityAvoidanceInSupport(comXYVelocity, comHeightDataToCorrect, zCurrentInWorld, pelvisZUpFrame);
      }
   }

   private void updateFractionOfSingularityAvoidanceToUse(YoDouble alphaToUpdate, double percentForMaximum)
   {
      double percentToStart = percentForMaximum - percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();
      double alpha = (desiredPercentOfLegLength.getDoubleValue() - percentToStart) / percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue();
      alphaToUpdate.set(MathTools.clamp(alpha, 0.0, 1.0));
   }

   private void smoothTransitionOutOfSingularityAvoidanceInSupport(CoMHeightTimeDerivativesData comHeightDataToCorrect)
   {
      heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
      heightVelocityCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightVelocity());
      heightAccelerationCorrectedFilteredForSingularityAvoidance.update(comHeightDataToCorrect.getComHeightAcceleration());

      comHeightDataToCorrect.setComHeight(desiredCenterOfMassHeightPoint.getReferenceFrame(), heightCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      comHeightDataToCorrect.setComHeightVelocity(heightVelocityCorrectedFilteredForSingularityAvoidance.getDoubleValue());
      comHeightDataToCorrect.setComHeightAcceleration(heightAccelerationCorrectedFilteredForSingularityAvoidance.getDoubleValue());

      // If the filtered desired com height caught up to the input one, then stop doing smooth transition.
      if (MathTools.epsilonEquals(desiredCenterOfMassHeightPoint.getZ() - heightCorrectedFilteredForSingularityAvoidance.getDoubleValue(), 0.0, 5e-3))
      {
         alphaSupportSingularityAvoidance.set(0.0);
         isSupportSingularityAvoidanceUsed.set(false);
         doSmoothTransitionOutOfSingularityAvoidance.set(false);
      }

      // If height is lower than filtered and the knee is bent enough, then really want to get out of singularity avoidance faster. So in this case, smooth faster...
      else if (desiredCenterOfMassHeightPoint.getZ() <= heightCorrectedFilteredForSingularityAvoidance.getDoubleValue() && (
            desiredPercentOfLegLength.getDoubleValue()
            < maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() - percentOfLegLengthMarginToEnableSingularityAvoidance.getDoubleValue()))
      {
         // Call this twice here to smooth faster. Need to get out of singularity avoidance!
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());
         heightCorrectedFilteredForSingularityAvoidance.update(desiredCenterOfMassHeightPoint.getZ());

         // If leg is bent a lot and singularity avoidance no longer needed, stop smoothing...
         if (desiredPercentOfLegLength.getDoubleValue()
             < maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() - percentOfLegLengthMarginToAbortSingularityAvoidance.getDoubleValue())
         {
            alphaSupportSingularityAvoidance.set(0.0);
            isSupportSingularityAvoidanceUsed.set(false);
            doSmoothTransitionOutOfSingularityAvoidance.set(false);
         }
      }
   }

   private final FrameVector2DBasics comVelocity = new FrameVector2D();
   private void applySingularityAvoidanceInSupport(FrameVector2DReadOnly comXYVelocity,
                                                   CoMHeightTimeDerivativesData comHeightDataToCorrect,
                                                   double zCurrent,
                                                   ReferenceFrame pelvisZUpFrame)
   {
      updateFractionOfSingularityAvoidanceToUse(alphaSupportSingularityAvoidance, maxPercentOfLegLengthForSingularityAvoidanceInSupport.getDoubleValue());

      double desiredOrMaxLegLength = Math.min(desiredLegLength.getDoubleValue(),
                                              maxPercentOfLegLengthForSingularityAvoidanceInSupport.getDoubleValue() * maximumLegLength.getDoubleValue());
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
}
