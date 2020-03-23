package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
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
   private final YoDouble alphaSwingKneeMechanicalLimitAvoidance;
   private final YoDouble alphaUnreachableFootstep;

   private final YoDouble maximumLegLength;

   private final YoDouble percentOfLegLengthThresholdToEnableSingularityAvoidance;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSwing;

   private final YoDouble desiredPercentOfLegLength;

   private final YoDouble desiredLegLength;
   private final YoDouble currentLegLength;

   private final RigidBodyBasics pelvis;

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

   private final YoDouble correctionAlphaFilter;

   private final YoDouble timeToCorrectForUnachievedSwingTranslation;
   private final AlphaFilteredYoVariable unachievedSwingTranslationFiltered;
   private final AlphaFilteredYoVariable unachievedSwingVelocityFiltered;
   private final AlphaFilteredYoVariable unachievedSwingAccelerationFiltered;

   public WorkspaceLimiterControlModule(String namePrefix,
                                        ContactablePlaneBody contactablePlaneBody,
                                        final RobotSide robotSide,
                                        WalkingControllerParameters walkingControllerParameters,
                                        final HighLevelHumanoidControllerToolbox controllerToolbox,
                                        YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      unachievedSwingTranslation = new YoFrameVector3D("unachievedSwingTranslation", worldFrame, registry);
      unachievedSwingVelocity = new YoFrameVector3D("unachievedSwingVelocity", worldFrame, registry);
      unachievedSwingAcceleration = new YoFrameVector3D("unachievedSwingAcceleration", worldFrame, registry);

      maximumLegLength = new YoDouble(namePrefix + "MaxLegLength", registry);
      maximumLegLength.set(walkingControllerParameters.getMaximumLegLengthForSingularityAvoidance());

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      frameBeforeHipPitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getFrameBeforeJoint();
      endEffectorFrame = contactablePlaneBody.getFrameAfterParentJoint();

      checkVelocityForSwingSingularityAvoidance = new YoBoolean(namePrefix + "CheckVelocityForSwingSingularityAvoidance", registry);

      alphaSwingSingularityAvoidance = new YoDouble(namePrefix + "AlphaSwingSingularityAvoidance", registry);
      alphaSwingKneeMechanicalLimitAvoidance = new YoDouble(namePrefix + "AlphaSwingKneeMechanicalLimitAvoidance", registry);
      alphaUnreachableFootstep = new YoDouble(namePrefix + "AlphaUnreachableFootstep", registry);
      alphaUnreachableFootstep.set(0.25);

      timeToCorrectForUnachievedSwingTranslation = new YoDouble(namePrefix + "TimeToCorrectForUnachievedSwingTranslation", registry);
      timeToCorrectForUnachievedSwingTranslation.set(0.2);

      unachievedSwingTranslationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingTranslationFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingVelocityFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingVelocityFiltered", registry, alphaUnreachableFootstep);
      unachievedSwingAccelerationFiltered = new AlphaFilteredYoVariable(namePrefix + "UnachievedSwingAccelerationFiltered", registry, alphaUnreachableFootstep);

      percentOfLegLengthThresholdToEnableSingularityAvoidance = new YoDouble(namePrefix + "PercThresSingularityAvoidance", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwing", registry);

      percentOfLegLengthThresholdToEnableSingularityAvoidance.set(0.87);
      maxPercentOfLegLengthForSingularityAvoidanceInSwing.set(0.97);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      useSingularityAvoidanceInSwing = new BooleanParameter(namePrefix + "UseSingularityAvoidanceInSwing",
                                                            registry,
                                                            swingTrajectoryParameters.useSingularityAvoidanceInSwing());

      correctionAlphaFilter = new YoDouble(namePrefix + "CorrectionAlphaFilter", registry);
      correctionAlphaFilter.set(0.98);

      desiredPercentOfLegLength = new YoDouble(namePrefix + "DesiredPercentOfLegLength", registry);

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
      yoDesiredFootLinearAcceleration = new YoFrameVector3D(namePrefix + "DesiredFootLinearAcceleration", worldFrame, registry);
      yoCorrectedDesiredFootLinearVelocity = new YoFrameVector3D(namePrefix + "CorrectedDesiredFootLinearVelocity", worldFrame, registry);
      yoCorrectedDesiredFootLinearAcceleration = new YoFrameVector3D(namePrefix + "CorrectedDesiredFootLinearAcceleration", worldFrame, registry);
      yoDesiredFootPosition.setToNaN();
      yoCorrectedDesiredFootPosition.setToNaN();
      yoDesiredFootLinearVelocity.setToNaN();
      yoCorrectedDesiredFootLinearVelocity.setToNaN();

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
   }

   public void resetHeightCorrectionParameters()
   {
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

      alphaSwingKneeMechanicalLimitAvoidance.set(0.0);

      yoDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);
      yoDesiredFootLinearAcceleration.set(desiredFootLinearAccelerationToCorrect);

      anklePosition.setToZero(endEffectorFrame);
      yoCurrentFootPosition.setMatchingFrame(anklePosition);
      anklePosition.changeFrame(virtualLegTangentialFrameHipCentered);
      currentLegLength.set(-anklePosition.getZ());

      desiredFootPosition.setIncludingFrame(desiredFootPositionToCorrect);
      desiredFootLinearAcceleration.setIncludingFrame(desiredFootLinearAccelerationToCorrect);
      desiredFootLinearVelocity.setIncludingFrame(desiredFootLinearVelocityToCorrect);

      desiredFootPosition.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearAcceleration.changeFrame(virtualLegTangentialFrameHipCentered);
      desiredFootLinearVelocity.changeFrame(virtualLegTangentialFrameHipCentered);

      desiredLegLength.set(-desiredFootPosition.getZ());
      desiredPercentOfLegLength.set(desiredLegLength.getDoubleValue() / maximumLegLength.getDoubleValue());

      if (useSingularityAvoidanceInSwing.getValue())
      {
         correctSwingFootTrajectoryForSingularityAvoidance(desiredFootPositionToCorrect,
                                                           desiredFootLinearVelocityToCorrect,
                                                           desiredFootLinearAccelerationToCorrect);
      }
   }

   private void correctSwingFootTrajectoryForSingularityAvoidance(FixedFramePoint3DBasics desiredFootPositionToCorrect,
                                                                  FixedFrameVector3DBasics desiredFootLinearVelocityToCorrect,
                                                                  FixedFrameVector3DBasics desiredFootLinearAccelerationToCorrect)
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

      alphaSwingSingularityAvoidance.set(
            (desiredPercentOfLegLength.getDoubleValue() - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue()) / (
                  maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue()
                  - percentOfLegLengthThresholdToEnableSingularityAvoidance.getDoubleValue()));
      alphaSwingSingularityAvoidance.set(MathTools.clamp(alphaSwingSingularityAvoidance.getDoubleValue(), 0.0, 1.0));

      double desiredOrMaxLegLength = -Math.min(desiredLegLength.getDoubleValue(),
                                               maxPercentOfLegLengthForSingularityAvoidanceInSwing.getDoubleValue() * maximumLegLength.getDoubleValue());

      // Mix the desired leg extension velocity to progressively follow the pelvis velocity as the the leg is more straight
      double desiredLinearVelocityZ = InterpolationTools.linearInterpolate(alphaSwingSingularityAvoidance.getDoubleValue(),
                                                                           desiredFootLinearVelocity.getZ(),
                                                                           pelvisLinearVelocity.getZ());
      double desiredLinearAccelerationZ = InterpolationTools.linearInterpolate(alphaSwingSingularityAvoidance.getDoubleValue(),
                                                                               desiredFootLinearAcceleration.getZ(),
                                                                               0.0);

      desiredFootPosition.setZ(desiredOrMaxLegLength);
      desiredFootLinearVelocity.setZ(desiredLinearVelocityZ);
      desiredFootLinearAcceleration.setZ(desiredLinearAccelerationZ);

      desiredFootPositionToCorrect.setMatchingFrame(desiredFootPosition);
      desiredFootLinearVelocityToCorrect.setMatchingFrame(desiredFootLinearVelocity);
      desiredFootLinearAccelerationToCorrect.setMatchingFrame(desiredFootLinearAcceleration);

      yoCorrectedDesiredFootPosition.set(desiredFootPositionToCorrect);
      yoCorrectedDesiredFootLinearVelocity.set(desiredFootLinearVelocityToCorrect);
      yoCorrectedDesiredFootLinearAcceleration.set(desiredFootLinearAccelerationToCorrect);

      unachievedSwingTranslation.sub(yoDesiredFootPosition, yoCorrectedDesiredFootPosition);
      unachievedSwingVelocity.sub(yoDesiredFootLinearVelocity, yoCorrectedDesiredFootLinearVelocity);
      unachievedSwingAcceleration.sub(yoDesiredFootLinearAcceleration, yoCorrectedDesiredFootLinearAcceleration);

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
}
