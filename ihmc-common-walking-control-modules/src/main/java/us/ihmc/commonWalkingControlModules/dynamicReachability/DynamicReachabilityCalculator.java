package us.ihmc.commonWalkingControlModules.dynamicReachability;

import java.util.ArrayList;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface;
import us.ihmc.commonWalkingControlModules.configurations.DynamicReachabilityParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class DynamicReachabilityCalculator
{
   //// TODO: 3/21/17 add in the ability to angle the hip forward for reachability
   //// TODO: 3/21/17 add in the ability to drop the pelvis for reachability

   private static final boolean USE_CONSERVATIVE_REQUIRED_ADJUSTMENT = true;
   private static final boolean VISUALIZE = false;
   private static final boolean VISUALIZE_REACHABILITY = false;

   private static final double epsilon = 0.005;
   private static final double stanceLegLengthToeOffFactor = 1.02;

   private static final double gradientThresholdForConsideration = 0.005;

   private final double transferTwiddleSizeDuration;
   private final double swingTwiddleSizeDuration;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble requiredAdjustmentSafetyFactor = new YoDouble("requiredAdjustmentSafetyFactor", registry);
   private final YoDouble requiredAdjustmentFeedbackGain = new YoDouble("requiredAdjustmentFeedbackGain", registry);
   private final YoDouble widthOfReachableRegion = new YoDouble("widthOfReachableRegion", registry);

   private final YoDouble minimumLegLength = new YoDouble("minimumLegLength", registry);
   private final YoDouble maximumLegLength = new YoDouble("maximumLegLength", registry);

   private final YoDouble maximumDesiredKneeBend = new YoDouble("maximumDesiredKneeBendForReachability", registry);

   private final YoDouble stanceLegMinimumHeight = new YoDouble("stanceLegMinimumHeight", registry);
   private final YoDouble stanceLegMaximumHeight = new YoDouble("stanceLegMaximumHeight", registry);
   private final YoDouble swingLegMinimumHeight = new YoDouble("swingLegMinimumHeight", registry);
   private final YoDouble swingLegMaximumHeight = new YoDouble("swingLegMaximumHeight", registry);

   private final YoBoolean isStepReachable = new YoBoolean("isStepReachable", registry);
   private final YoBoolean isModifiedStepReachable = new YoBoolean("isModifiedStepReachable", registry);

   private final YoInteger numberOfIterations = new YoInteger("numberOfTimingAdjustmentIterations", registry);
   private final YoInteger numberOfAdjustments = new YoInteger("numberOfCoMAdjustments", registry);
   private final YoInteger maximumNumberOfAdjustments = new YoInteger("maxNumberOfCoMAdjustments", registry);

   private final YoDouble currentTransferAdjustment = new YoDouble("currentTransferAdjustment", registry);
   private final YoDouble currentSwingAdjustment = new YoDouble("currentSwingAdjustment", registry);
   private final YoDouble nextTransferAdjustment = new YoDouble("nextTransferAdjustment", registry);

   private final ArrayList<YoDouble> higherSwingAdjustments = new ArrayList<>();
   private final ArrayList<YoDouble> higherTransferAdjustments = new ArrayList<>();

   private final SideDependentList<YoFramePoint3D> hipMinimumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> hipMaximumLocations = new SideDependentList<>();

   private final ArrayList<YoDouble> requiredParallelCoMAdjustments = new ArrayList<>();
   private final ArrayList<YoDouble> achievedParallelCoMAdjustments = new ArrayList<>();

   private final YoDouble currentTransferAlpha = new YoDouble("currentTransferAlpha", registry);
   private final YoDouble currentSwingAlpha = new YoDouble("currentSwingAlpha", registry);
   private final YoDouble nextTransferAlpha = new YoDouble("nextTransferAlpha", registry);

   private final ExecutionTimer reachabilityTimer = new ExecutionTimer("reachabilityTimer", registry);

   private final FrameVector2D currentInitialTransferGradient = new FrameVector2D(worldFrame);
   private final FrameVector2D currentEndTransferGradient = new FrameVector2D(worldFrame);

   private final FrameVector2D currentInitialSwingGradient = new FrameVector2D(worldFrame);
   private final FrameVector2D currentEndSwingGradient = new FrameVector2D(worldFrame);

   private final FrameVector2D nextInitialTransferGradient = new FrameVector2D(worldFrame);
   private final FrameVector2D nextEndTransferGradient = new FrameVector2D(worldFrame);

   private final ArrayList<FrameVector2D> higherSwingGradients = new ArrayList<>();
   private final ArrayList<FrameVector2D> higherTransferGradients = new ArrayList<>();

   private final SideDependentList<FramePoint3D> ankleLocations = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();
   private final SideDependentList<FramePoint3D> adjustedAnkleLocations = new SideDependentList<>();
   private final SideDependentList<FrameVector3D> hipOffsets = new SideDependentList<>();

   private final SideDependentList<YoFramePoint3D> yoAnkleLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> yoHipLocations = new SideDependentList<>();

   private Footstep nextFootstep;
   private boolean isInTransfer;

   private final LineSegment1D stanceHeightLine = new LineSegment1D();
   private final LineSegment1D stepHeightLine = new LineSegment1D();

   private final ReferenceFrame predictedCoMFrame;
   private final TranslationReferenceFrame predictedPelvisFrame;
   private final SideDependentList<ReferenceFrame> predictedHipFrames = new SideDependentList<>();
   private final SideDependentList<Vector2dZUpFrame> stepDirectionFrames = new SideDependentList<>();

   private final FramePoint3D adjustedCoMPosition = new FramePoint3D();
   private final FramePoint3D predictedCoMPosition = new FramePoint3D();

   private final FrameQuaternion predictedPelvisOrientation = new FrameQuaternion();
   private final FrameQuaternion stanceFootOrientation = new FrameQuaternion();
   private final FrameQuaternion footstepAnkleOrientation = new FrameQuaternion();

   private final FrameVector3D tempGradient = new FrameVector3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint3D tempPoint3d = new FramePoint3D();
   private final FramePoint3D tempFinalCoM = new FramePoint3D();

   private final DynamicReachabilityParameters dynamicReachabilityParameters;
   private final double thighLength;
   private final double shinLength;
   private final double maximumKneeBend;

   private final TimeAdjustmentSolver solver;

   private final ICPPlannerInterface icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private final TDoubleArrayList originalTransferDurations = new TDoubleArrayList();
   private final TDoubleArrayList originalSwingDurations = new TDoubleArrayList();
   private final TDoubleArrayList originalTransferAlphas = new TDoubleArrayList();
   private final TDoubleArrayList originalSwingAlphas = new TDoubleArrayList();

   public DynamicReachabilityCalculator(ICPPlannerInterface icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
                                        DynamicReachabilityParameters dynamicReachabilityParameters, YoVariableRegistry parentRegistry,
                                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.dynamicReachabilityParameters = dynamicReachabilityParameters;
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      this.requiredAdjustmentSafetyFactor.set(dynamicReachabilityParameters.getRequiredAdjustmentSafetyFactor());
      this.requiredAdjustmentFeedbackGain.set(dynamicReachabilityParameters.getRequiredAdjustmentFeedbackGain());

      this.transferTwiddleSizeDuration = dynamicReachabilityParameters.getPercentOfTransferDurationToCalculateGradient();
      this.swingTwiddleSizeDuration = dynamicReachabilityParameters.getPercentOfSwingDurationToCalculateGradient();

      this.maximumDesiredKneeBend.set(dynamicReachabilityParameters.getMaximumDesiredKneeBend());
      this.maximumNumberOfAdjustments.set(dynamicReachabilityParameters.getMaximumNumberOfCoMAdjustments());

      maximumKneeBend = Math.min(Math.min(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getJointLimitUpper(),
            fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).getJointLimitUpper()), 1.7);

      solver = new TimeAdjustmentSolver(icpPlanner.getNumberOfFootstepsToConsider(), dynamicReachabilityParameters);

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleLocations.put(robotSide, new FramePoint3D());
         adjustedAnkleLocations.put(robotSide, new FramePoint3D());
         hipOffsets.put(robotSide, new FrameVector3D());

         String prefix = robotSide.getShortLowerCaseName();
         if (VISUALIZE_REACHABILITY)
         {
            yoAnkleLocations.put(robotSide, new YoFramePoint3D(prefix + "AnkleLocation", worldFrame, registry));
            yoHipLocations.put(robotSide, new YoFramePoint3D(prefix + "HipLocation", worldFrame, registry));
         }

         YoFramePoint3D hipMaximumLocation = new YoFramePoint3D(prefix + "PredictedHipMaximumPoint", worldFrame, registry);
         YoFramePoint3D hipMinimumLocation = new YoFramePoint3D(prefix + "PredictedHipMinimumPoint", worldFrame, registry);
         hipMaximumLocations.put(robotSide, hipMaximumLocation);
         hipMinimumLocations.put(robotSide, hipMinimumLocation);
         
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         ReferenceFrame ankleFrame = fullRobotModel.getFoot(robotSide).getParentJoint().getFrameAfterJoint();
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }

      int numberOfFootstepsToConsider = icpPlanner.getNumberOfFootstepsToConsider();
      for (int i = 0; i < numberOfFootstepsToConsider - 3; i++)
      {
         FrameVector2D higherSwingGradient = new FrameVector2D(worldFrame);
         FrameVector2D higherTransferGradient = new FrameVector2D(worldFrame);
         higherSwingGradients.add(higherSwingGradient);
         higherTransferGradients.add(higherTransferGradient);

         YoDouble higherSwingAdjustment = new YoDouble("higherSwingAdjustment" + i, registry);
         YoDouble higherTransferAdjustment = new YoDouble("higherTransferAdjustment" + i, registry);
         higherSwingAdjustments.add(higherSwingAdjustment);
         higherTransferAdjustments.add(higherTransferAdjustment);
      }

      for (int i = 0; i < dynamicReachabilityParameters.getMaximumNumberOfCoMAdjustments(); i++)
      {
         requiredParallelCoMAdjustments.add(new YoDouble("requiredParallelCoMAdjustment" + i, registry));
         achievedParallelCoMAdjustments.add(new YoDouble("achievedParallelCoMAdjustment" + i, registry));
      }


      // compute leg segment lengths
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
      FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      shinLength = kneePoint.distance(anklePoint);

      // setup reference frames
      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePoint3D pelvis = new FramePoint3D(pelvisFrame);
      FramePoint3D com = new FramePoint3D(centerOfMassFrame);
      pelvis.changeFrame(centerOfMassFrame);
      FrameVector3D translationToCoM = new FrameVector3D(centerOfMassFrame);
      translationToCoM.set(com);
      translationToCoM.sub(pelvis);
      translationToCoM.changeFrame(pelvisFrame);

      predictedCoMFrame = new ReferenceFrame("Predicted CoM Position", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            predictedCoMPosition.changeFrame(worldFrame);
            predictedPelvisOrientation.changeFrame(worldFrame);
            transformToParent.setTranslation(predictedCoMPosition);
            transformToParent.setRotation(predictedPelvisOrientation);
         }
      };

      predictedPelvisFrame = new TranslationReferenceFrame("Predicted Pelvis Frame", predictedCoMFrame);
      predictedPelvisFrame.updateTranslation((Vector3DReadOnly) translationToCoM);

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameVector3D translationToPelvis = new FrameVector3D(pelvisFrame);
         FramePoint3D pelvisCenter = new FramePoint3D(pelvisFrame);
         FramePoint3D hipJoint = new FramePoint3D(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getFrameAfterJoint());
         hipJoint.changeFrame(pelvisFrame);
         translationToPelvis.set(hipJoint);
         translationToPelvis.sub(pelvisCenter);
         TranslationReferenceFrame predictedHipFrame = new TranslationReferenceFrame(robotSide.getShortLowerCaseName() + " Predicted Hip Frame",
               predictedPelvisFrame);
         predictedHipFrame.updateTranslation((Vector3DReadOnly) translationToPelvis);
         predictedHipFrames.put(robotSide, predictedHipFrame);

         Vector2dZUpFrame stepDirectionFrame = new Vector2dZUpFrame(robotSide.getShortLowerCaseName() + "Step Direction Frame", worldFrame);
         stepDirectionFrames.put(robotSide, stepDirectionFrame);
      }

      updateLegLengthLimits();
      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());

      for (RobotSide side : RobotSide.values)
      {
         YoFramePoint3D hipMaximumLocation = hipMaximumLocations.get(side);
         YoFramePoint3D hipMinimumLocation = hipMinimumLocations.get(side);

         YoGraphicPosition hipMaximumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Maximum Hip Point", hipMaximumLocation,
               0.01, YoAppearance.ForestGreen());
         YoGraphicPosition hipMinimumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Minimum Hip Point", hipMinimumLocation,
               0.01, YoAppearance.Blue());

         yoGraphicsList.add(hipMaximumLocationViz);
         yoGraphicsList.add(hipMinimumLocationViz);
      }

      yoGraphicsList.setVisible(VISUALIZE);

      if (VISUALIZE_REACHABILITY)
      {
         YoGraphicsList reachabilityGraphicsList = new YoGraphicsList(getClass().getSimpleName() + "Reachability");

         double minLength = minimumLegLength.getDoubleValue();
         double maxLength = maximumLegLength.getDoubleValue();

         for (RobotSide robotSide : RobotSide.values)
         {
            YoGraphicPosition ballInside = new YoGraphicPosition(robotSide.getShortLowerCaseName() + "BallInside", yoAnkleLocations.get(robotSide), minLength, YoAppearance.Green());
            YoGraphicPosition ballOutside = new YoGraphicPosition(robotSide.getShortLowerCaseName() + "BallOutside", yoAnkleLocations.get(robotSide), maxLength, YoAppearance.LightGreen());

            YoGraphicPosition footViz = new YoGraphicPosition("foot", yoAnkleLocations.get(robotSide), 0.075, YoAppearance.Blue());
            YoGraphicPosition hipViz = new YoGraphicPosition("hip", yoHipLocations.get(robotSide), 0.075, YoAppearance.Red());

            ballInside.getAppearance().setTransparency(0.8);
            ballOutside.getAppearance().setTransparency(0.95);

            reachabilityGraphicsList.add(ballInside);
            reachabilityGraphicsList.add(ballOutside);
            reachabilityGraphicsList.add(footViz);
            reachabilityGraphicsList.add(hipViz);
         }

         yoGraphicsListRegistry.registerYoGraphicsList(reachabilityGraphicsList);
      }


      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   private void updateFrames(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);

      predictedCoMPosition.setToZero(worldFrame);
      predictedCoMPosition.set(tempFinalCoM);

      stanceFootOrientation.setToZero(fullRobotModel.getFoot(stanceSide).getBodyFixedFrame());
      nextFootstep.getAnkleOrientation(footstepAnkleOrientation, transformsFromAnkleToSole.get(nextFootstep.getRobotSide()));

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      stanceFootOrientation.changeFrame(pelvisFrame);
      footstepAnkleOrientation.changeFrame(pelvisFrame);
      predictedPelvisOrientation.setToZero(pelvisFrame);
      predictedPelvisOrientation.interpolate(stanceFootOrientation, footstepAnkleOrientation, 0.5);

      FramePoint3D stanceAnkleLocation = ankleLocations.get(stanceSide);
      FramePoint3D upcomingStepLocation = ankleLocations.get(swingSide);
      stanceAnkleLocation.setToZero(fullRobotModel.getLegJoint(stanceSide, LegJointName.ANKLE_PITCH).getFrameAfterJoint());
      nextFootstep.getAnklePosition(upcomingStepLocation, transformsFromAnkleToSole.get(nextFootstep.getRobotSide()));
      upcomingStepLocation.changeFrame(worldFrame);
      stanceAnkleLocation.changeFrame(worldFrame);
      if (VISUALIZE_REACHABILITY) yoAnkleLocations.get(swingSide).set(upcomingStepLocation);
      if (VISUALIZE_REACHABILITY) yoAnkleLocations.get(stanceSide).set(stanceAnkleLocation);

      predictedCoMFrame.update();
      predictedPelvisFrame.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         predictedHipFrames.get(robotSide).update();
         if (VISUALIZE_REACHABILITY)
         {
            tempPoint.setToZero(predictedHipFrames.get(robotSide));
            tempPoint.changeFrame(worldFrame);
            yoHipLocations.get(robotSide).set(tempPoint);
            yoHipLocations.get(robotSide).setZ(0.8);
         }
      }
   }

   private void updateLegLengthLimits()
   {
      this.maximumLegLength.set(thighLength + shinLength);

      double minimumLegLength = computeLegLength(thighLength, shinLength, maximumDesiredKneeBend.getDoubleValue());
      this.minimumLegLength.set(minimumLegLength);
   }

   private static double computeLegLength(double thighLength, double shinLength, double kneeAngle)
   {
      double minimumLegLength = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) + 2.0 * thighLength * shinLength * Math.cos(kneeAngle);
      minimumLegLength = Math.sqrt(minimumLegLength);

      return minimumLegLength;
   }

   private void computeHeightLineFromStance(RobotSide supportSide, double minimumStanceLegLength, double maximumStanceLegLength)
   {
      FramePoint3D ankleLocation = ankleLocations.get(supportSide);
      ankleLocation.changeFrame(worldFrame);
      tempPoint3d.set(ankleLocation);

      // get the hip location in XY
      tempPoint.setToZero(predictedHipFrames.get(supportSide));
      tempPoint.changeFrame(worldFrame);
      tempFinalCoM.setIncludingFrame(tempPoint);

      tempFinalCoM.changeFrame(worldFrame);
      hipMaximumLocations.get(supportSide).set(tempFinalCoM);
      hipMinimumLocations.get(supportSide).set(tempFinalCoM);

      double planarDistance = tempFinalCoM.distance(tempPoint3d);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumStanceLegLength)
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumStanceLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumStanceLegLength)
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumStanceLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         maximumHeight += ankleLocation.getZ();
      }

      hipMaximumLocations.get(supportSide).setZ(maximumHeight);
      hipMinimumLocations.get(supportSide).setZ(minimumHeight);

      stanceLegMinimumHeight.set(minimumHeight);
      stanceLegMaximumHeight.set(maximumHeight);
      stanceHeightLine.set(minimumHeight, maximumHeight);
   }

   private void computeHeightLineFromStep(Footstep nextFootstep, double minimumStepLegLength, double maximumStepLegLength)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();

      FramePoint3D ankleLocation = ankleLocations.get(swingSide);
      ankleLocation.changeFrame(worldFrame);
      tempPoint3d.set(ankleLocation);

      tempPoint.setToZero(predictedHipFrames.get(swingSide));
      tempPoint.changeFrame(worldFrame);
      tempFinalCoM.setIncludingFrame(tempPoint);

      double planarDistance = tempFinalCoM.distance(tempPoint3d);

      tempFinalCoM.changeFrame(worldFrame);
      ankleLocation.changeFrame(worldFrame);

      hipMaximumLocations.get(swingSide).set(tempFinalCoM);
      hipMinimumLocations.get(swingSide).set(tempFinalCoM);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumStepLegLength)
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumStepLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumStepLegLength)
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumStepLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         maximumHeight += ankleLocation.getZ();
      }

      hipMaximumLocations.get(swingSide).setZ(maximumHeight);
      hipMinimumLocations.get(swingSide).setZ(minimumHeight);

      swingLegMinimumHeight.set(minimumHeight);
      swingLegMaximumHeight.set(maximumHeight);
      stepHeightLine.set(minimumHeight, maximumHeight);
   }

   private void reset()
   {
      numberOfAdjustments.set(0);

      originalTransferDurations.clear();
      originalTransferAlphas.clear();
      originalSwingDurations.clear();
      originalSwingAlphas.clear();

      currentTransferAdjustment.set(0.0);
      currentSwingAdjustment.set(0.0);
      nextTransferAdjustment.set(0.0);

      currentTransferAlpha.setToNaN();
      currentSwingAlpha.setToNaN();
      nextTransferAlpha.setToNaN();

      currentInitialTransferGradient.setToNaN();
      currentEndTransferGradient.setToNaN();
      currentInitialSwingGradient.setToNaN();
      currentEndSwingGradient.setToNaN();
      nextInitialTransferGradient.setToNaN();
      nextEndTransferGradient.setToNaN();

      for (int i = 0; i < higherSwingAdjustments.size(); i++)
      {
         higherSwingGradients.get(i).setToNaN();
         higherTransferGradients.get(i).setToNaN();

         higherSwingAdjustments.get(i).setToNaN();
         higherTransferAdjustments.get(i).setToNaN();
      }

      for (int i = 0; i < requiredParallelCoMAdjustments.size(); i++)
      {
         requiredParallelCoMAdjustments.get(i).setToNaN();
         achievedParallelCoMAdjustments.get(i).setToNaN();
      }
   }


   /**
    * Sets the location of the next footstep in the plan
    * @param nextFootstep next desired footstep location
    */
   public void setUpcomingFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }


   /**
    * Indicates that the robot is starting the transfer phase.
    */
   public void setInTransfer()
   {
      isInTransfer = true;
   }

   /**
    * Indicates that the robot is starting the swing phase.
    */
   public void setInSwing()
   {
      isInTransfer = false;
   }

   /**
    * Checks whether the current footstep is reachable given the desired footstep timing.
    *
    * @return reachable or not
    */
   public boolean checkReachabilityOfStep()
   {
      boolean isStepReachable = checkReachabilityInternal();

      this.isStepReachable.set(isStepReachable);

      return isStepReachable;
   }

   /**
    * Checks whether the current footstep is reachable given the desired footstep timing. If it is, does nothing. If it is not, modifies the
    * ICP Plan timing to make sure that is is.
    */
   public void verifyAndEnsureReachability()
   {
      reachabilityTimer.startMeasurement();
      reset();

      // Efficiently checks the reachability by examining if the required heights of the stance hip and step hip overlap, as this determines reachability.
      boolean isStepReachable = checkReachabilityInternal();
      this.isStepReachable.set(isStepReachable);
      this.isModifiedStepReachable.set(isStepReachable);

      if (!isStepReachable)
      { // The step isn't reachable using the efficient checks.
         // Compute the required amount of adjustment in the direction of the step. This is less efficient but a little more accurate than the previous method,
         // so we don't necessarily want to use it by default. If the adjustment is within a certain small bound, we say it is reachable and exit the algorithm.
         double originalRequiredAdjustment = computeRequiredAdjustment();
         double requiredAdjustment = originalRequiredAdjustment;
         isStepReachable = MathTools.intervalContains(requiredAdjustment, -epsilon, epsilon);

         if (isStepReachable)
         {
            this.isStepReachable.set(true);
            this.isModifiedStepReachable.set(true);
            return;
         }

         // Compute the number of higher order steps that should be considered in the optimization algorithm. In practice, we aren't really looking at these,
         // as they don't actually have much of an effect, as their gradient is so small.
         int numberOfHigherSteps = computeNumberOfHigherSteps();

         // Record the original timing durations and alphas that were submitted to the ICP Planner.
         for (int i = 0; i < 1 + numberOfHigherSteps; i++)
         {
            originalTransferDurations.add(icpPlanner.getTransferDuration(i));
            originalTransferAlphas.add(icpPlanner.getTransferDurationAlpha(i));
            originalSwingDurations.add(icpPlanner.getSwingDuration(i));
            originalSwingAlphas.add(icpPlanner.getSwingDurationAlpha(i));
         }
         originalTransferDurations.add(icpPlanner.getTransferDuration(numberOfHigherSteps + 1));
         originalTransferAlphas.add(icpPlanner.getTransferDurationAlpha(numberOfHigherSteps + 1));

         // Compute the gradient associated with adjusting the different time segments.
         computeGradients(numberOfHigherSteps);
         // Submit the gradient information to the solver
         submitGradientInformationToSolver(numberOfHigherSteps);

         while(!isStepReachable)
         { // Start a loop for the solver to find the necessary timing adjustments to achieved the desired CoM adjustment.
            if (numberOfAdjustments.getIntegerValue() >= maximumNumberOfAdjustments.getIntegerValue() )
               break;

            // Set the desired adjustment for the solver to achieve.
            solver.setDesiredParallelAdjustment(requiredAdjustment);

            requiredParallelCoMAdjustments.get(numberOfAdjustments.getIntegerValue()).set(requiredAdjustment);

            // Compute the required timing adjustments to achieved the desired CoM adjustment using the linear approximation of the gradient.
            try
            {
               solver.compute();
               numberOfIterations.set(solver.getNumberOfIterations());
            }
            catch (NoConvergenceException e)
            {
               e.printStackTrace();
               PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once. "
                     + "The Timing optimization solver failed on iteration " + numberOfAdjustments.getIntegerValue() + ", so sticking with the last solution.");
               break;
            }

            // Extract the adjustment solutions from the solver.
            extractTimingSolutionsFromSolver(numberOfHigherSteps);

            // Apply the adjustment solutions to the ICP Planner.
            submitTimingAdjustmentsToPlanner(numberOfHigherSteps);

            // Compute the remaining CoM adjustment to be reachable using the new times.
            initializePlan(tempFinalCoM);
            updateFrames(nextFootstep);
            double remainingAdjustment = computeRequiredAdjustment();
            isStepReachable = MathTools.intervalContains(remainingAdjustment, -epsilon, epsilon);

            // Compute the achieved CoM adjustment from the remaining adjustment using the new times.
            double achievedAdjustment;
            if (Math.signum(remainingAdjustment) != Math.signum(originalRequiredAdjustment))
            {
               if (Math.signum(remainingAdjustment) < 0.0)
                  achievedAdjustment = originalRequiredAdjustment + widthOfReachableRegion.getDoubleValue() - remainingAdjustment;
               else
                  achievedAdjustment = originalRequiredAdjustment - widthOfReachableRegion.getDoubleValue() + remainingAdjustment;
            }
            else
            {
               achievedAdjustment = originalRequiredAdjustment - remainingAdjustment;
            }
            achievedParallelCoMAdjustments.get(numberOfAdjustments.getIntegerValue()).set(achievedAdjustment);

            // Compute the adjustment we would like the solver to try and achieve on the next iteration.
            requiredAdjustment += requiredAdjustmentFeedbackGain.getDoubleValue() * (originalRequiredAdjustment - achievedAdjustment);

            isModifiedStepReachable.set(isStepReachable);
            numberOfAdjustments.increment();
         }

         computeTimingAdjustmentsForController(numberOfHigherSteps);
      }

      reachabilityTimer.stopMeasurement();
   }

   public boolean wasTimingAdjusted()
   {
      return numberOfAdjustments.getIntegerValue() > 0;
   }

   private boolean checkReachabilityInternal()
   {
      RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

      updateFrames(nextFootstep);
      updateLegLengthLimits();

      double heightChange = computeChangeInHeight(nextFootstep);

      double minimumStanceLegLength, minimumStepLegLength;
      if (heightChange > dynamicReachabilityParameters.getThresholdForStepUp())
      { // stepping up, so we should allow the upcoming stance leg length to bend as much as we want
         minimumStepLegLength = computeLegLength(thighLength, shinLength, maximumKneeBend);
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
      }
      else if (heightChange < dynamicReachabilityParameters.getThresholdForStepDown())
      { // stepping down, so we should allow the upcoming swing leg to bend as much as we want
         minimumStanceLegLength = computeLegLength(thighLength, shinLength, maximumKneeBend);
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }
      else
      {
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }

      computeHeightLineFromStance(supportSide, minimumStanceLegLength, stanceLegLengthToeOffFactor * maximumLegLength.getDoubleValue());
      computeHeightLineFromStep(nextFootstep, minimumStepLegLength, maximumLegLength.getDoubleValue());

      return stanceHeightLine.isOverlappingExclusive(stepHeightLine);
   }

   private double computeChangeInHeight(Footstep footstep)
   {
      RobotSide stanceSide = footstep.getRobotSide().getOppositeSide();

      FramePoint3D stanceAnkleLocation = ankleLocations.get(stanceSide);
      FramePoint3D stepAnkleLocation = ankleLocations.get(stanceSide.getOppositeSide());
      stanceAnkleLocation.changeFrame(worldFrame);
      stepAnkleLocation.changeFrame(worldFrame);

      return stepAnkleLocation.getZ() - stanceAnkleLocation.getZ();
   }


   private double computeRequiredAdjustment()
   {
      RobotSide stepSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = stepSide.getOppositeSide();

      ReferenceFrame stanceHipFrame = predictedHipFrames.get(stanceSide);
      ReferenceFrame stepHipFrame = predictedHipFrames.get(stepSide);
      Vector2dZUpFrame stepDirectionFrame = stepDirectionFrames.get(stanceSide);

      // compute base point of upcoming sphere account for hip offsets
      FramePoint3D upcomingAnklePoint = ankleLocations.get(stepSide);
      FramePoint3D stanceAnklePoint = ankleLocations.get(stanceSide);
      FramePoint3D adjustedUpcomingAnklePoint = adjustedAnkleLocations.get(stepSide);
      FramePoint3D adjustedStanceAnklePoint = adjustedAnkleLocations.get(stanceSide);

      FrameVector3D stepHipVector = hipOffsets.get(stepSide);
      FrameVector3D stanceHipVector = hipOffsets.get(stanceSide);

      tempPoint.setToZero(stepHipFrame);
      tempPoint.changeFrame(predictedCoMFrame);
      stepHipVector.setIncludingFrame(tempPoint);
      stepHipVector.changeFrame(worldFrame);
      tempPoint.setToZero(stanceHipFrame);
      tempPoint.changeFrame(predictedCoMFrame);
      stanceHipVector.setIncludingFrame(tempPoint);
      stanceHipVector.changeFrame(worldFrame);

      // compute step direction frame accounting for hip offsets
      adjustedUpcomingAnklePoint.setIncludingFrame(upcomingAnklePoint);
      adjustedUpcomingAnklePoint.changeFrame(worldFrame);
      adjustedUpcomingAnklePoint.sub(stepHipVector);

      adjustedStanceAnklePoint.setIncludingFrame(stanceAnklePoint);
      adjustedStanceAnklePoint.changeFrame(worldFrame);
      adjustedStanceAnklePoint.sub(stanceHipVector);

      tempVector.setIncludingFrame(adjustedUpcomingAnklePoint);
      tempVector.sub(adjustedStanceAnklePoint);
      stepDirectionFrame.setXAxis(tempVector);

      // compute the actual planar step direction
      tempVector.changeFrame(stepDirectionFrame);
      double stepHeight = tempVector.getZ();
      double stepDistance = tempVector.getX();

      // compute the minimum leg lengths
      double minimumStanceLegLength, minimumStepLegLength;
      if (stepHeight > dynamicReachabilityParameters.getThresholdForStepUp())
      {
         minimumStepLegLength = computeLegLength(thighLength, shinLength, maximumKneeBend);
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
      }
      else if (stepHeight < dynamicReachabilityParameters.getThresholdForStepDown())
      {
         minimumStanceLegLength = computeLegLength(thighLength, shinLength, maximumKneeBend);
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }
      else
      {
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }

      double minimumStanceHipPosition, maximumStepHipPosition;
      if (USE_CONSERVATIVE_REQUIRED_ADJUSTMENT)
      {
         minimumStanceHipPosition = SphereIntersectionTools.computeDistanceToCenterOfIntersectionEllipse(stepDistance, stepHeight,
               minimumStanceLegLength, maximumLegLength.getDoubleValue());
         maximumStepHipPosition = SphereIntersectionTools.computeDistanceToCenterOfIntersectionEllipse(stepDistance, stepHeight,
               stanceLegLengthToeOffFactor * maximumLegLength.getDoubleValue(), minimumStepLegLength);
      }
      else
      {
         minimumStanceHipPosition = SphereIntersectionTools.computeDistanceToNearEdgeOfIntersectionEllipse(stepDistance, stepHeight,
               minimumStanceLegLength, maximumLegLength.getDoubleValue());
         maximumStepHipPosition = SphereIntersectionTools.computeDistanceToFarEdgeOfIntersectionEllipse(stepDistance, stepHeight,
               stanceLegLengthToeOffFactor * maximumLegLength.getDoubleValue(), minimumStepLegLength);
      }

      tempPoint.setToZero(predictedCoMFrame);
      tempPoint.changeFrame(worldFrame);
      tempPoint.sub(adjustedStanceAnklePoint);
      tempPoint.changeFrame(stepDirectionFrame);

      widthOfReachableRegion.set(maximumStepHipPosition);
      widthOfReachableRegion.sub(minimumStanceHipPosition);

      double requiredAdjustment;
      double safetyMultiplier = requiredAdjustmentSafetyFactor.getDoubleValue();

      if (tempPoint.getX() > maximumStepHipPosition)
      {
         requiredAdjustment = (maximumStepHipPosition - tempPoint.getX()) - safetyMultiplier * widthOfReachableRegion.getDoubleValue();
      }
      else if (tempPoint.getX() < minimumStanceHipPosition)
      {
         requiredAdjustment = (minimumStanceHipPosition - tempPoint.getX()) + safetyMultiplier * widthOfReachableRegion.getDoubleValue();
      }
      else
      {
         requiredAdjustment = 0.0;
      }

      return requiredAdjustment;
   }

   private int computeNumberOfHigherSteps()
   {
      if (dynamicReachabilityParameters.useHigherOrderSteps())
      {
         int numberOfFootstepsToConsider = icpPlanner.getNumberOfFootstepsToConsider();
         int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

         return Math.min(numberOfFootstepsToConsider - 3, numberOfFootstepsRegistered - 1);
      }
      else
      {
         return 0;
      }
   }

   private void extractTimingSolutionsFromSolver(int numberOfHigherSteps)
   {
      // handle current transfer
      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();

      double currentInitialTransferDuration = originalTransferAlphas.get(0) * originalTransferDurations.get(0);
      currentInitialTransferDuration += currentInitialTransferAdjustment;

      double currentEndTransferDuration = (1.0 - originalTransferAlphas.get(0)) * originalTransferDurations.get(0);
      currentEndTransferDuration += currentEndTransferAdjustment;

      currentTransferAdjustment.set(currentInitialTransferAdjustment + currentEndTransferAdjustment);
      currentTransferAlpha.set(currentInitialTransferDuration / (currentInitialTransferDuration + currentEndTransferDuration));

      // handle current swing
      double currentInitialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double currentEndSwingAdjustment = solver.getCurrentEndSwingAdjustment();

      double currentSwingInitialDuration = originalSwingAlphas.get(0) * originalSwingDurations.get(0);
      currentSwingInitialDuration += currentInitialSwingAdjustment;

      double currentSwingEndDuration = (1.0 - originalSwingAlphas.get(0)) * originalSwingDurations.get(0);
      currentSwingEndDuration += currentEndSwingAdjustment;

      currentSwingAdjustment.set(currentInitialSwingAdjustment + currentEndSwingAdjustment);
      currentSwingAlpha.set(currentSwingInitialDuration / (currentSwingInitialDuration + currentSwingEndDuration));

      // handle next transfer
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      double nextInitialTransferDuration = originalTransferAlphas.get(1) * originalTransferDurations.get(1);
      nextInitialTransferDuration += nextInitialTransferAdjustment;

      double nextEndTransferDuration = (1.0 - originalTransferAlphas.get(1)) * originalTransferDurations.get(1);
      nextEndTransferDuration += nextEndTransferAdjustment;

      nextTransferAdjustment.set(nextInitialTransferAdjustment + nextEndTransferAdjustment);
      this.nextTransferAlpha.set(nextInitialTransferDuration / (nextInitialTransferDuration + nextEndTransferDuration));

      // handle higher values
      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         double higherSwingAdjustment = solver.getHigherSwingAdjustment(i);
         double higherTransferAdjustment = solver.getHigherTransferAdjustment(i);

         higherSwingAdjustments.get(i).set(higherSwingAdjustment);
         higherTransferAdjustments.get(i).set(higherTransferAdjustment);
      }
   }

   private void submitTimingAdjustmentsToPlanner(int numberOfHigherSteps)
   {
      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      icpPlanner.setTransferDuration(0, originalTransferDurations.get(0) + currentTransferAdjustment.getDoubleValue());
      icpPlanner.setTransferDurationAlpha(0, currentTransferAlpha.getDoubleValue());

      icpPlanner.setSwingDuration(0, originalSwingDurations.get(0) + currentSwingAdjustment.getDoubleValue());
      icpPlanner.setSwingDurationAlpha(0, currentSwingAlpha.getDoubleValue());

      boolean isThisTheFinalTransfer = (numberOfFootstepsRegistered == 1);

      double adjustedTransferDuration = originalTransferDurations.get(1) + nextTransferAdjustment.getDoubleValue();
      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(adjustedTransferDuration);
         icpPlanner.setFinalTransferDurationAlpha(nextTransferAlpha.getDoubleValue());
      }
      else
      {
         icpPlanner.setTransferDuration(1, adjustedTransferDuration);
         icpPlanner.setTransferDurationAlpha(1, nextTransferAlpha.getDoubleValue());
      }

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         double swingDuration = originalSwingDurations.get(i + 1);
         double swingAdjustment = higherSwingAdjustments.get(i).getDoubleValue();
         icpPlanner.setSwingDuration(i + 1, swingDuration + swingAdjustment);

         int transferIndex = i + 2;
         double transferDuration = originalTransferDurations.get(transferIndex);
         double transferAdjustment = higherTransferAdjustments.get(i).getDoubleValue();

         isThisTheFinalTransfer = (numberOfFootstepsRegistered == transferIndex);
         if (isThisTheFinalTransfer)
            icpPlanner.setFinalTransferDuration(transferDuration + transferAdjustment);
         else
            icpPlanner.setTransferDuration(transferIndex, transferDuration + transferAdjustment);
      }
   }

   private double transferDuration;
   private double swingDuration;
   private double finalTransferDuration;
   private double nextTransferDuration;

   private void computeTimingAdjustmentsForController(int numberOfHigherSteps)
   {
      finalTransferDuration = Double.NaN;
      nextTransferDuration = Double.NaN;

      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      transferDuration = originalTransferDurations.get(0) + currentTransferAdjustment.getDoubleValue();
      swingDuration = originalSwingDurations.get(0) + currentSwingAdjustment.getDoubleValue();

      boolean isThisTheFinalTransfer = (numberOfFootstepsRegistered == 1);

      double adjustedTransferDuration = originalTransferDurations.get(1) + nextTransferAdjustment.getDoubleValue();
      if (isThisTheFinalTransfer)
      {
         finalTransferDuration = adjustedTransferDuration;
      }
      else
      {
         nextTransferDuration = adjustedTransferDuration;
      }

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         int transferIndex = i + 2;
         double transferDuration = originalTransferDurations.get(transferIndex);
         double transferAdjustment = higherTransferAdjustments.get(i).getDoubleValue();

         isThisTheFinalTransfer = (numberOfFootstepsRegistered == transferIndex);
         if (isThisTheFinalTransfer)
         {
            finalTransferDuration = transferDuration + transferAdjustment;
         }
      }
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration;
   }

   public double getNextTransferDuration()
   {
      return nextTransferDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   private void computeGradients(int numberOfHigherSteps)
   {
      computeCurrentTransferGradient();
      computeCurrentSwingGradient();
      computeNextTransferGradient();

      for (int stepIndex = 0; stepIndex < numberOfHigherSteps; stepIndex++)
      {
         computeHigherSwingGradient(stepIndex + 1);
         computeHigherTransferGradient(stepIndex + 2);
      }
   }

   private void computeCurrentTransferGradient()
   {
      int stepNumber = 0;

      double currentTransferDuration = originalTransferDurations.get(stepNumber);
      double currentTransferDurationAlpha = originalTransferAlphas.get(stepNumber);

      double currentInitialTransferDuration = currentTransferDurationAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferDurationAlpha) * currentTransferDuration;

      // compute initial transfer duration gradient
      double variation = -transferTwiddleSizeDuration * currentInitialTransferDuration;
      double modifiedTransferDurationAlpha = (currentInitialTransferDuration + variation) / (currentTransferDuration + variation);

      submitTransferTiming(stepNumber, currentTransferDuration + variation, modifiedTransferDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentInitialTransferGradient.set(tempGradient);

      // compute end transfer duration gradient
      variation = -transferTwiddleSizeDuration * currentEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (currentEndTransferDuration + variation) / (currentTransferDuration + variation);

      submitTransferTiming(stepNumber, currentTransferDuration + variation, modifiedTransferDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentEndTransferGradient.set(tempGradient);

      // reset timing
      submitTransferTiming(stepNumber, currentTransferDuration, currentTransferDurationAlpha);
   }

   private void computeNextTransferGradient()
   {
      int stepNumber = 1;

      double nextTransferDuration = originalTransferDurations.get(stepNumber);
      double nextTransferDurationAlpha = originalTransferAlphas.get(stepNumber);

      if (nextTransferDuration == 0.0)
      {
         nextEndTransferGradient.setToZero();
         nextInitialTransferGradient.setToZero();
         return;
      }

      double nextInitialTransferDuration = nextTransferDurationAlpha * nextTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferDurationAlpha) * nextTransferDuration;

      // compute initial transfer duration gradient
      double variation = -transferTwiddleSizeDuration * nextInitialTransferDuration;
      double modifiedTransferDurationAlpha = (nextInitialTransferDuration + variation) / (nextTransferDuration + variation);

      submitTransferTiming(stepNumber, nextTransferDuration + variation, modifiedTransferDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextInitialTransferGradient.set(tempGradient);

      // compute end transfer duration gradient
      variation = -transferTwiddleSizeDuration * nextEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (nextEndTransferDuration + variation) / (nextTransferDuration + variation);

      submitTransferTiming(stepNumber, nextTransferDuration + variation, modifiedTransferDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextEndTransferGradient.set(tempGradient);

      // reset timing
      submitTransferTiming(stepNumber, nextTransferDuration, nextTransferDurationAlpha);
   }

   private void computeHigherTransferGradient(int stepIndex)
   {
      double originalDuration = originalTransferDurations.get(stepIndex);
      double variation = -transferTwiddleSizeDuration * originalDuration;

      submitTransferTiming(stepIndex, originalDuration + variation, -1.0);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      higherTransferGradients.get(stepIndex - 2).set(tempGradient);

      // reset timing
      submitTransferTiming(stepIndex, originalDuration, -1.0);
   }

   private void computeCurrentSwingGradient()
   {
      int stepNumber = 0;

      double currentSwingDuration = originalSwingDurations.get(stepNumber);
      double currentSwingDurationAlpha = originalSwingAlphas.get(stepNumber);

      double currentInitialSwingDuration = currentSwingDurationAlpha * currentSwingDuration;
      double currentEndSwingDuration = (1.0 - currentSwingDurationAlpha) * currentSwingDuration;

      // compute initial swing duration gradient
      double variation = -swingTwiddleSizeDuration * currentInitialSwingDuration;
      double modifiedSwingDurationAlpha = (currentInitialSwingDuration + variation) / (currentSwingDuration + variation);

      submitSwingTiming(stepNumber, currentSwingDuration + variation, modifiedSwingDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentInitialSwingGradient.set(tempGradient);

      // compute end swing duration gradient
      variation = -swingTwiddleSizeDuration * currentEndSwingDuration;
      modifiedSwingDurationAlpha = 1.0 - (currentEndSwingDuration + variation) / (currentSwingDuration + variation);

      submitSwingTiming(stepNumber, currentSwingDuration + variation, modifiedSwingDurationAlpha);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentEndSwingGradient.set(tempGradient);

      // reset timing
      submitSwingTiming(stepNumber, currentSwingDuration, currentSwingDurationAlpha);
   }

   private void computeHigherSwingGradient(int stepIndex)
   {
      double duration = originalSwingDurations.get(stepIndex);
      double variation = -swingTwiddleSizeDuration * duration;

      submitSwingTiming(stepIndex, duration + variation, -1.0);
      applyVariation(adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      higherSwingGradients.get(stepIndex - 1).set(tempGradient);

      //reset timing
      submitSwingTiming(stepIndex, duration, -1.0);
   }

   private void submitTransferTiming(int stepNumber, double duration, double alpha)
   {
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == stepNumber;

      if (alpha != -1.0)
      {
         if (isThisTheFinalTransfer)
            icpPlanner.setFinalTransferDurationAlpha(alpha);
         else
            icpPlanner.setTransferDurationAlpha(stepNumber, alpha);
      }

      if (isThisTheFinalTransfer)
         icpPlanner.setFinalTransferDuration(duration);
      else
         icpPlanner.setTransferDuration(stepNumber, duration);
   }

   private void submitSwingTiming(int stepNumber, double duration, double alpha)
   {
      if (alpha != -1.0)
         icpPlanner.setSwingDurationAlpha(stepNumber, alpha);

      icpPlanner.setSwingDuration(stepNumber, duration);
   }

   private void applyVariation(FramePoint3D comToPack)
   {
      if (isInTransfer)
         icpPlanner.computeFinalCoMPositionInTransfer();
      else
         icpPlanner.computeFinalCoMPositionInSwing();

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }

   private void initializePlan(FramePoint3D comToPack)
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      if (isInTransfer)
         icpPlanner.initializeForTransfer(currentInitialTime);
      else
         icpPlanner.initializeForSingleSupport(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }


   private void computeGradient(FramePoint3D originalPosition, FramePoint3D adjustedPosition, double variation, FrameVector3D gradientToPack)
   {
      originalPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(originalPosition);
      tempPoint.setZ(0.0);
      gradientToPack.setToZero(worldFrame);
      gradientToPack.set(adjustedPosition);
      gradientToPack.sub(tempPoint);
      gradientToPack.scale(1.0 / variation);

      if (MathTools.intervalContains(gradientToPack.getX(), gradientThresholdForConsideration))
         gradientToPack.setX(0.0);
      if (MathTools.intervalContains(gradientToPack.getY(), gradientThresholdForConsideration))
         gradientToPack.setY(0.0);
   }

   private void submitGradientInformationToSolver(int numberOfHigherSteps)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();

      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      solver.setNumberOfFootstepsToConsider(icpPlanner.getNumberOfFootstepsToConsider());
      solver.setNumberOfFootstepsRegistered(numberOfFootstepsRegistered);

      solver.reshape();

      extractGradient(currentInitialTransferGradient, stanceSide, tempGradient);
      solver.setCurrentInitialTransferGradient(tempGradient);

      extractGradient(currentEndTransferGradient, stanceSide, tempGradient);
      solver.setCurrentEndTransferGradient(tempGradient);

      extractGradient(currentInitialSwingGradient, stanceSide, tempGradient);
      solver.setCurrentInitialSwingGradient(tempGradient);

      extractGradient(currentEndSwingGradient, stanceSide, tempGradient);
      solver.setCurrentEndSwingGradient(tempGradient);

      extractGradient(nextInitialTransferGradient, stanceSide, tempGradient);
      solver.setNextInitialTransferGradient(tempGradient);

      extractGradient(nextEndTransferGradient, stanceSide, tempGradient);
      solver.setNextEndTransferGradient(tempGradient);

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         extractGradient(higherSwingGradients.get(i), stanceSide, tempGradient);
         solver.setHigherSwingGradient(i, tempGradient);

         extractGradient(higherTransferGradients.get(i), stanceSide, tempGradient);
         solver.setHigherTransferGradient(i, tempGradient);
      }

      // define bounds and timing constraints
      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      double currentTransferAlpha = icpPlanner.getTransferDurationAlpha(0);

      double currentSwingDuration = icpPlanner.getSwingDuration(0);
      double currentSwingAlpha = icpPlanner.getSwingDurationAlpha(0);

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      double nextTransferAlpha = icpPlanner.getTransferDurationAlpha(1);

      solver.setCurrentTransferDuration(currentTransferDuration, currentTransferAlpha);
      solver.setCurrentSwingDuration(currentSwingDuration, currentSwingAlpha);
      solver.setNextTransferDuration(nextTransferDuration, nextTransferAlpha);

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         double swingDuration = icpPlanner.getSwingDuration(i + 1);
         double transferDuration = icpPlanner.getTransferDuration(i + 2);

         solver.setHigherSwingDuration(i, swingDuration);
         solver.setHigherTransferDuration(i, transferDuration);
      }
   }

   private void extractGradient(FrameVector2D gradientToExtract, RobotSide stanceSide, FrameVector3D gradientToPack)
   {
      gradientToPack.setToZero(worldFrame);
      gradientToPack.set(gradientToExtract);
      gradientToPack.changeFrame(stepDirectionFrames.get(stanceSide));
   }




   private static class Vector2dZUpFrame extends ReferenceFrame
   {
      private final FrameVector2D xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2D(parentFrame);
      }

      public void setXAxis(FrameVector3D xAxis)
      {
         xAxis.changeFrame(getParent());
         this.xAxis.set(xAxis);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumns(x, y, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
