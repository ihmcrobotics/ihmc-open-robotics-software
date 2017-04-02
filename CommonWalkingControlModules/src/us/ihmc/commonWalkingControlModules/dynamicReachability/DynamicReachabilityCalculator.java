package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;

public class DynamicReachabilityCalculator
{
   //// TODO: 3/21/17 add in the ability to angle the hip forward for reachability
   //// TODO: 3/21/17 add in the ability to drop the pelvis for reachability

   private static final boolean COMPUTE_ACHIEVED_ADJUSTMENT = true;

   private static final boolean USE_HIGHER_ORDER_STEPS = true;
   private static final boolean USE_CONSERVATIVE_REQUIRED_ADJUSTMENT = true;
   private static final boolean VISUALIZE = true;
   private static final double MAXIMUM_DESIRED_KNEE_BEND = 0.2;
   private static final double MAXIMUM_KNEE_BEND = 1.7;
   private static final double STEP_UP_THRESHOLD = 0.05;
   private static final double STEP_DOWN_THRESHOLD = -0.05;
   private static final int MAXIMUM_NUMBER_OF_ADJUSTMENTS = 3;

   private static final double transferTwiddleSizeDuration = 0.2;
   private static final double swingTwiddleSizeDuration = 0.2;

   private static final double requiredAdjustmentSF = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable requiredAdjustmentSafetyFactor = new DoubleYoVariable("requiredAdjustmentSafetyFactor", registry);
   private final DoubleYoVariable minimumLegLength = new DoubleYoVariable("minimumLegLength", registry);
   private final DoubleYoVariable maximumLegLength = new DoubleYoVariable("maximumLegLength", registry);

   private final DoubleYoVariable maximumDesiredKneeBend = new DoubleYoVariable("maximumDesiredKneeBendForReachability", registry);

   private final DoubleYoVariable stanceLegMinimumHeight = new DoubleYoVariable("stanceLegMinimumHeight", registry);
   private final DoubleYoVariable stanceLegMaximumHeight = new DoubleYoVariable("stanceLegMaximumHeight", registry);
   private final DoubleYoVariable swingLegMinimumHeight = new DoubleYoVariable("swingLegMinimumHeight", registry);
   private final DoubleYoVariable swingLegMaximumHeight = new DoubleYoVariable("swingLegMaximumHeight", registry);

   private final BooleanYoVariable reachableWRTStanceFoot = new BooleanYoVariable("reachableWRTStanceFoot", registry);
   private final BooleanYoVariable reachableWRTFootstep = new BooleanYoVariable("reachableWRTFootstep", registry);
   private final BooleanYoVariable isStepReachable = new BooleanYoVariable("isStepReachable", registry);
   private final BooleanYoVariable isModifiedStepReachable = new BooleanYoVariable("isModifiedStepReachable", registry);

   private final IntegerYoVariable numberOfAdjustments = new IntegerYoVariable("numberOfCoMAdjustmentIterations", registry);
   private final IntegerYoVariable maximumNumberOfAdjustments = new IntegerYoVariable("maxNumberOfCoMAdjustmentIterations", registry);

   private final DoubleYoVariable yoCurrentTransferAdjustment = new DoubleYoVariable("currentTransferAdjustment", registry);
   private final DoubleYoVariable yoCurrentSwingAdjustment = new DoubleYoVariable("currentSwingAdjustment", registry);
   private final DoubleYoVariable yoNextTransferAdjustment = new DoubleYoVariable("nextTransferAdjustment", registry);

   private final ArrayList<DoubleYoVariable> higherSwingAdjustments = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> higherTransferAdjustments = new ArrayList<>();

   private final SideDependentList<YoFramePoint> hipMinimumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> hipMaximumLocations = new SideDependentList<>();

   private final ArrayList<DoubleYoVariable> requiredParallelCoMAdjustments = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> achievedParallelCoMAdjustments = new ArrayList<>();

   private final DoubleYoVariable currentTransferAlpha = new DoubleYoVariable("currentTransferAlpha", registry);
   private final DoubleYoVariable currentSwingAlpha = new DoubleYoVariable("currentSwingAlpha", registry);
   private final DoubleYoVariable nextTransferAlpha = new DoubleYoVariable("nextTransferAlpha", registry);

   private final FrameVector2d currentInitialTransferGradient = new FrameVector2d(worldFrame);
   private final FrameVector2d currentEndTransferGradient = new FrameVector2d(worldFrame);

   private final FrameVector2d currentInitialSwingGradient = new FrameVector2d(worldFrame);
   private final FrameVector2d currentEndSwingGradient = new FrameVector2d(worldFrame);

   private final FrameVector2d nextInitialTransferGradient = new FrameVector2d(worldFrame);
   private final FrameVector2d nextEndTransferGradient = new FrameVector2d(worldFrame);

   private final ArrayList<FrameVector2d> higherSwingGradients = new ArrayList<>();
   private final ArrayList<FrameVector2d> higherTransferGradients = new ArrayList<>();

   private double currentTransferAdjustment;
   private double currentSwingAdjustment;
   private double nextTransferAdjustment;

   private final SideDependentList<FramePoint> ankleLocations = new SideDependentList<>();
   private final SideDependentList<FramePoint> adjustedAnkleLocations = new SideDependentList<>();
   private final SideDependentList<FrameVector> hipOffsets = new SideDependentList<>();

   private Footstep nextFootstep;
   private boolean isInTransfer;

   private final LineSegment1d stanceHeightLine = new LineSegment1d();
   private final LineSegment1d stepHeightLine = new LineSegment1d();

   private final ReferenceFrame predictedCoMFrame;
   private final TranslationReferenceFrame predictedPelvisFrame;
   private final SideDependentList<ReferenceFrame> predictedHipFrames = new SideDependentList<>();
   private final SideDependentList<Vector2dZUpFrame> stepDirectionFrames = new SideDependentList<>();

   private final FramePoint2d adjustedCoMPosition = new FramePoint2d();
   private final FramePoint predictedCoMPosition = new FramePoint();

   private final FrameOrientation predictedPelvisOrientation = new FrameOrientation();
   private final FrameOrientation stanceFootOrientation = new FrameOrientation();
   private final FrameOrientation footstepOrientation = new FrameOrientation();

   private final FrameVector tempGradient = new FrameVector();
   private final FrameVector tempVector = new FrameVector();

   private final FramePoint tempPoint = new FramePoint();
   private final FramePoint2d tempPoint2d = new FramePoint2d();
   private final FramePoint2d tempFinalCoM = new FramePoint2d();

   private final double thighLength;
   private final double shinLength;

   private final TimeAdjustmentSolver solver;

   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(icpPlanner, fullRobotModel, centerOfMassFrame, MAXIMUM_DESIRED_KNEE_BEND, parentRegistry, yoGraphicsListRegistry);
   }

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         double maximumDesiredKneeBend, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      this.requiredAdjustmentSafetyFactor.set(requiredAdjustmentSF);
      this.maximumDesiredKneeBend.set(maximumDesiredKneeBend);
      maximumNumberOfAdjustments.set(MAXIMUM_NUMBER_OF_ADJUSTMENTS);

      solver = new TimeAdjustmentSolver(icpPlanner.getNumberOfFootstepsToConsider(), USE_HIGHER_ORDER_STEPS, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleLocations.put(robotSide, new FramePoint());
         adjustedAnkleLocations.put(robotSide, new FramePoint());
         hipOffsets.put(robotSide, new FrameVector());

         YoFramePoint hipMaximumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMaximumPoint", worldFrame, registry);
         YoFramePoint hipMinimumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMinimumPoint", worldFrame, registry);
         hipMaximumLocations.put(robotSide, hipMaximumLocation);
         hipMinimumLocations.put(robotSide, hipMinimumLocation);
      }

      int numberOfFootstepsToConsider = icpPlanner.getNumberOfFootstepsToConsider();
      for (int i = 0; i < numberOfFootstepsToConsider - 3; i++)
      {
         FrameVector2d higherSwingGradient = new FrameVector2d(worldFrame);
         FrameVector2d higherTransferGradient = new FrameVector2d(worldFrame);
         higherSwingGradients.add(higherSwingGradient);
         higherTransferGradients.add(higherTransferGradient);

         DoubleYoVariable higherSwingAdjustment = new DoubleYoVariable("higherSwingAdjustment" + i, registry);
         DoubleYoVariable higherTransferAdjustment = new DoubleYoVariable("higherTransferAdjustment" + i, registry);
         higherSwingAdjustments.add(higherSwingAdjustment);
         higherTransferAdjustments.add(higherTransferAdjustment);
      }

      for (int i = 0; i < MAXIMUM_NUMBER_OF_ADJUSTMENTS; i++)
      {
         requiredParallelCoMAdjustments.add(new DoubleYoVariable("requiredParallelCoMAdjustment" + i, registry));
         achievedParallelCoMAdjustments.add(new DoubleYoVariable("achievedParallelCoMAdjustment" + i, registry));
      }


      // compute leg segment lengths
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint hipPoint = new FramePoint(hipPitchFrame);
      FramePoint kneePoint = new FramePoint(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint anklePoint = new FramePoint(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      shinLength = kneePoint.distance(anklePoint);

      // setup reference frames
      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePoint pelvis = new FramePoint(pelvisFrame);
      FramePoint com = new FramePoint(centerOfMassFrame);
      pelvis.changeFrame(centerOfMassFrame);
      FrameVector translationToCoM = new FrameVector(centerOfMassFrame);
      translationToCoM.set(com);
      translationToCoM.sub(pelvis);
      translationToCoM.changeFrame(pelvisFrame);

      predictedCoMFrame = new ReferenceFrame("Predicted CoM Position", worldFrame, false, false, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            predictedCoMPosition.changeFrame(worldFrame);
            predictedPelvisOrientation.changeFrame(worldFrame);
            transformToParent.setTranslation(predictedCoMPosition.getPoint());
            transformToParent.setRotation(predictedPelvisOrientation.getQuaternion());
         }
      };

      predictedPelvisFrame = new TranslationReferenceFrame("Predicted Pelvis Frame", predictedCoMFrame);
      predictedPelvisFrame.updateTranslation(translationToCoM.getVector());

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameVector translationToPelvis = new FrameVector(pelvisFrame);
         FramePoint pelvisCenter = new FramePoint(pelvisFrame);
         FramePoint hipJoint = new FramePoint(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getFrameAfterJoint());
         hipJoint.changeFrame(pelvisFrame);
         translationToPelvis.set(hipJoint);
         translationToPelvis.sub(pelvisCenter);
         TranslationReferenceFrame predictedHipFrame = new TranslationReferenceFrame(robotSide.getShortLowerCaseName() + " Predicted Hip Frame", predictedPelvisFrame);
         predictedHipFrame.updateTranslation(translationToPelvis.getVector());
         predictedHipFrames.put(robotSide, predictedHipFrame);

         Vector2dZUpFrame stepDirectionFrame = new Vector2dZUpFrame(robotSide.getShortLowerCaseName() + "Step Direction Frame", worldFrame);
         stepDirectionFrames.put(robotSide, stepDirectionFrame);
      }

      updateLegLengthLimits();
      setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());

      for (RobotSide side : RobotSide.values)
      {
         YoFramePoint hipMaximumLocation = hipMaximumLocations.get(side);
         YoFramePoint hipMinimumLocation = hipMinimumLocations.get(side);

         YoGraphicPosition hipMaximumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Maximum Hip Point", hipMaximumLocation, 0.01, YoAppearance.ForestGreen());
         YoGraphicPosition hipMinimumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Minimum Hip Point", hipMinimumLocation, 0.01, YoAppearance.Blue());

         yoGraphicsList.add(hipMaximumLocationViz);
         yoGraphicsList.add(hipMinimumLocationViz);
      }

      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   private void updateFrames(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);
      if (tempFinalCoM.containsNaN())
         throw new RuntimeException("Final CoM Contains NaN!");

      predictedCoMPosition.setToZero(worldFrame);
      predictedCoMPosition.setXY(tempFinalCoM);

      stanceFootOrientation.setToZero(fullRobotModel.getFoot(stanceSide).getBodyFixedFrame());
      nextFootstep.getOrientationIncludingFrame(footstepOrientation);

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      stanceFootOrientation.changeFrame(pelvisFrame);
      footstepOrientation.changeFrame(pelvisFrame);
      predictedPelvisOrientation.setToZero(pelvisFrame);
      predictedPelvisOrientation.interpolate(stanceFootOrientation, footstepOrientation, 0.5);

      FramePoint stanceAnkleLocation = ankleLocations.get(stanceSide);
      FramePoint upcomingStepLocation = ankleLocations.get(swingSide);
      stanceAnkleLocation.setToZero(fullRobotModel.getLegJoint(stanceSide, LegJointName.ANKLE_PITCH).getFrameAfterJoint());
      nextFootstep.getPositionIncludingFrame(upcomingStepLocation);
      upcomingStepLocation.changeFrame(worldFrame);
      stanceAnkleLocation.changeFrame(worldFrame);

      predictedCoMFrame.update();
      predictedPelvisFrame.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         predictedHipFrames.get(robotSide).update();
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
      FramePoint ankleLocation = ankleLocations.get(supportSide);
      ankleLocation.changeFrame(worldFrame);
      ankleLocation.getFrameTuple2d(tempPoint2d);

      // get the hip location in XY
      tempPoint.setToZero(predictedHipFrames.get(supportSide));
      tempPoint.changeFrame(worldFrame);
      tempFinalCoM.setByProjectionOntoXYPlaneIncludingFrame(tempPoint);

      tempFinalCoM.changeFrame(worldFrame);
      hipMaximumLocations.get(supportSide).setXY(tempFinalCoM);
      hipMinimumLocations.get(supportSide).setXY(tempFinalCoM);

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumLegLength.getDoubleValue())
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumStanceLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
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

      FramePoint ankleLocation = ankleLocations.get(swingSide);
      ankleLocation.changeFrame(worldFrame);
      ankleLocation.getFrameTuple2d(tempPoint2d);

      tempPoint.setToZero(predictedHipFrames.get(swingSide));
      tempPoint.changeFrame(worldFrame);
      tempFinalCoM.setByProjectionOntoXYPlaneIncludingFrame(tempPoint);

      double planarDistance = tempFinalCoM.distance(tempPoint2d);

      tempFinalCoM.changeFrame(worldFrame);
      ankleLocation.changeFrame(worldFrame);

      hipMaximumLocations.get(swingSide).setXY(tempFinalCoM);
      hipMinimumLocations.get(swingSide).setXY(tempFinalCoM);

      double minimumHeight, maximumHeight;
      if (planarDistance >= minimumLegLength.getDoubleValue())
      {
         minimumHeight = 0.0;
      }
      else
      {
         minimumHeight = Math.sqrt(Math.pow(minimumStepLegLength, 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
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

      yoCurrentTransferAdjustment.set(0.0);
      yoCurrentSwingAdjustment.set(0.0);
      yoNextTransferAdjustment.set(0.0);

      currentTransferAdjustment = 0.0;
      currentSwingAdjustment = 0.0;
      nextTransferAdjustment = 0.0;

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

   private boolean caughtError = false;
   /**
    * Checks whether the current footstep is reachable given the desired footstep timing. If it is, does nothing. If it is not, modifies the
    * ICP Plan timing to make sure that is is.
    */
   public void verifyAndEnsureReachability()
   {
      reset();

      boolean isStepReachable = checkReachabilityInternal();

      this.reachableWRTStanceFoot.set(stanceHeightLine.length() > 0.0);
      this.reachableWRTFootstep.set(stepHeightLine.length() > 0.0);
      this.isStepReachable.set(isStepReachable);
      this.isModifiedStepReachable.set(isStepReachable);

      while (!isStepReachable)
      {
         if (numberOfAdjustments.getIntegerValue() >= maximumNumberOfAdjustments.getIntegerValue() )
            break;

         boolean needToMoveCoMBackward = (stanceHeightLine.getMaxPoint() <= stepHeightLine.getMinPoint());
         double requiredAdjustment = computeRequiredAdjustment(needToMoveCoMBackward);
         requiredParallelCoMAdjustments.get(numberOfAdjustments.getIntegerValue()).set(requiredAdjustment);

         requiredAdjustment *= requiredAdjustmentSafetyFactor.getDoubleValue();

         int numberOfHigherSteps = computeNumberOfHigherSteps();

         computeGradients(numberOfHigherSteps);
         submitGradientInformation(numberOfHigherSteps, requiredAdjustment);

         try
         {
            solver.compute();
            caughtError = false;
         }
         catch (NoConvergenceException e)
         {
            e.printStackTrace();
            PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
            caughtError = true;
         }

         if (caughtError)
            break;

         extractSolution(numberOfHigherSteps);
         applyAdjustments(numberOfHigherSteps);

         double initialTime = icpPlanner.getInitialTime();
         if (isInTransfer)
            icpPlanner.initializeForTransfer(initialTime);
         else
            icpPlanner.initializeForSingleSupport(initialTime);

         isStepReachable = checkReachabilityInternal();
         isModifiedStepReachable.set(isStepReachable);
         numberOfAdjustments.increment();

         if (COMPUTE_ACHIEVED_ADJUSTMENT)
         {
            double newRequiredAdjustment = computeRequiredAdjustment(needToMoveCoMBackward);
            achievedParallelCoMAdjustments.get(numberOfAdjustments.getIntegerValue() - 1).set(requiredAdjustment - newRequiredAdjustment);
         }
      }
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

      this.reachableWRTStanceFoot.set(stanceHeightLine.length() > 0.0);
      this.reachableWRTFootstep.set(stepHeightLine.length() > 0.0);
      this.isStepReachable.set(isStepReachable);

      return isStepReachable;
   }

   private boolean checkReachabilityInternal()
   {
      RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

      updateFrames(nextFootstep);
      updateLegLengthLimits();

      double heightChange = computeChangeInHeight(nextFootstep);

      double minimumStanceLegLength, minimumStepLegLength;
      if (heightChange > STEP_UP_THRESHOLD)
      {
         minimumStepLegLength = computeLegLength(thighLength, shinLength, MAXIMUM_KNEE_BEND);
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
      }
      else if (heightChange < STEP_DOWN_THRESHOLD)
      {
         minimumStanceLegLength = computeLegLength(thighLength, shinLength, MAXIMUM_KNEE_BEND);
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }
      else
      {
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
         minimumStepLegLength = minimumLegLength.getDoubleValue();
      }

      computeHeightLineFromStance(supportSide, minimumStanceLegLength, maximumLegLength.getDoubleValue());
      computeHeightLineFromStep(nextFootstep, minimumStepLegLength, maximumLegLength.getDoubleValue());

      return stanceHeightLine.isOverlappingExclusive(stepHeightLine);
   }

   private double computeChangeInHeight(Footstep footstep)
   {
      RobotSide stanceSide = footstep.getRobotSide().getOppositeSide();

      FramePoint stanceAnkleLocation = ankleLocations.get(stanceSide);
      FramePoint stepAnkleLocation = ankleLocations.get(stanceSide.getOppositeSide());
      stanceAnkleLocation.changeFrame(worldFrame);
      stepAnkleLocation.changeFrame(worldFrame);

      return stepAnkleLocation.getZ() - stanceAnkleLocation.getZ();
   }


   private double computeRequiredAdjustment(boolean needToMoveCoMBackward)
   {
      RobotSide stepSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = stepSide.getOppositeSide();

      ReferenceFrame stanceHipFrame = predictedHipFrames.get(stanceSide);
      ReferenceFrame stepHipFrame = predictedHipFrames.get(stepSide);
      Vector2dZUpFrame stepDirectionFrame = stepDirectionFrames.get(stanceSide);

      // compute base point of upcoming sphere account for hip offsets
      FramePoint upcomingAnklePoint = ankleLocations.get(stepSide);
      FramePoint stanceAnklePoint = ankleLocations.get(stanceSide);
      FramePoint adjustedUpcomingAnklePoint = adjustedAnkleLocations.get(stepSide);
      FramePoint adjustedStanceAnklePoint = adjustedAnkleLocations.get(stanceSide);

      FrameVector stepHipVector = hipOffsets.get(stepSide);
      FrameVector stanceHipVector = hipOffsets.get(stanceSide);

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
      if (stepHeight > STEP_UP_THRESHOLD)
      {
         minimumStepLegLength = computeLegLength(thighLength, shinLength, MAXIMUM_KNEE_BEND);
         minimumStanceLegLength = minimumLegLength.getDoubleValue();
      }
      else if (stepHeight < STEP_DOWN_THRESHOLD)
      {
         minimumStanceLegLength = computeLegLength(thighLength, shinLength, MAXIMUM_KNEE_BEND);
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
               maximumLegLength.getDoubleValue(), minimumStepLegLength);
      }
      else
      {
         minimumStanceHipPosition = SphereIntersectionTools.computeDistanceToNearEdgeOfIntersectionEllipse(stepDistance, stepHeight,
               minimumStanceLegLength, maximumLegLength.getDoubleValue());
         maximumStepHipPosition = SphereIntersectionTools.computeDistanceToFarEdgeOfIntersectionEllipse(stepDistance, stepHeight,
               maximumLegLength.getDoubleValue(), minimumStepLegLength);
      }

      tempPoint.setToZero(predictedCoMFrame);
      tempPoint.changeFrame(worldFrame);
      tempPoint.sub(adjustedStanceAnklePoint);
      tempPoint.changeFrame(stepDirectionFrame);

      if (needToMoveCoMBackward)
         return (maximumStepHipPosition - tempPoint.getX());
      else
         return (minimumStanceHipPosition - tempPoint.getX());
   }

   private int computeNumberOfHigherSteps()
   {
      if (USE_HIGHER_ORDER_STEPS)
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

   private void submitGradientInformation(int numberOfHigherSteps, double requiredAdjustment)
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

      solver.setDesiredParallelAdjustment(requiredAdjustment);

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

   private void extractSolution(int numberOfHigherSteps)
   {
      // handle current transfer
      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();

      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      double currentTransferAlpha = icpPlanner.getTransferDurationAlpha(0);
      double currentInitialTransferDuration = currentTransferAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferAlpha) * currentTransferDuration;

      currentInitialTransferDuration += currentInitialTransferAdjustment;
      currentEndTransferDuration += currentEndTransferAdjustment;
      currentTransferAdjustment = currentInitialTransferAdjustment + currentEndTransferAdjustment;
      this.currentTransferAlpha.set(currentInitialTransferDuration / (currentInitialTransferDuration + currentEndTransferDuration));

      // handle current swing
      double initialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double endSwingAdjustment = solver.getCurrentEndSwingAdjustment();

      double swingDuration = icpPlanner.getSwingDuration(0);
      double currentSwingAlpha = icpPlanner.getSwingDurationAlpha(0);
      double swingInitialDuration = currentSwingAlpha * swingDuration;
      double swingEndDuration = (1.0 - currentSwingAlpha) * swingDuration;

      swingInitialDuration += initialSwingAdjustment;
      swingEndDuration += endSwingAdjustment;
      currentSwingAdjustment = initialSwingAdjustment + endSwingAdjustment;
      this.currentSwingAlpha.set(swingInitialDuration / (swingInitialDuration + swingEndDuration));

      // handle next transfer
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      double nextTransferAlpha = icpPlanner.getTransferDurationAlpha(1);
      double nextInitialTransferDuration = nextTransferAlpha * currentTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferAlpha) * nextTransferDuration;

      nextInitialTransferDuration += nextInitialTransferAdjustment;
      nextEndTransferDuration += nextEndTransferAdjustment;
      nextTransferAdjustment = nextInitialTransferAdjustment + nextEndTransferAdjustment;
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

   private void applyAdjustments(int numberOfHigherSteps)
   {
      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      yoCurrentTransferAdjustment.add(currentTransferAdjustment);
      icpPlanner.setTransferDuration(0, currentTransferDuration + currentTransferAdjustment);
      icpPlanner.setTransferDurationAlpha(0, currentTransferAlpha.getDoubleValue());

      double currentSwingDuration = icpPlanner.getSwingDuration(0);
      yoCurrentSwingAdjustment.add(currentSwingAdjustment);
      icpPlanner.setSwingDuration(0, currentSwingDuration + currentSwingAdjustment);
      icpPlanner.setSwingDurationAlpha(0, currentSwingAlpha.getDoubleValue());

      boolean isThisTheFinalTransfer = (numberOfFootstepsRegistered == 1);

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      yoNextTransferAdjustment.add(nextTransferAdjustment);
      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(nextTransferDuration + nextTransferAdjustment);
         icpPlanner.setFinalTransferDurationAlpha(nextTransferAlpha.getDoubleValue());
      }
      else
      {
         icpPlanner.setTransferDuration(1, nextTransferDuration + nextTransferAdjustment);
         icpPlanner.setTransferDurationAlpha(1, nextTransferAlpha.getDoubleValue());
      }

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         double swingDuration = icpPlanner.getSwingDuration(i + 1);
         double swingAdjustment = higherSwingAdjustments.get(i).getDoubleValue();
         icpPlanner.setSwingDuration(i + 1, swingDuration + swingAdjustment);

         int transferIndex = i + 2;
         double transferDuration = icpPlanner.getTransferDuration(transferIndex);
         double transferAdjustment = higherTransferAdjustments.get(i).getDoubleValue();
         isThisTheFinalTransfer = (numberOfFootstepsRegistered == transferIndex);
         if (isThisTheFinalTransfer)
            icpPlanner.setFinalTransferDuration(transferDuration + transferAdjustment);
         else
            icpPlanner.setTransferDuration(transferIndex, transferDuration + transferAdjustment);
      }
   }

   private void computeCurrentTransferGradient()
   {
      int stepNumber = 0;

      double currentTransferDuration = icpPlanner.getTransferDuration(stepNumber);
      double currentTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(stepNumber);

      double currentInitialTransferDuration = currentTransferDurationAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferDurationAlpha) * currentTransferDuration;

      // compute initial transfer duration gradient
      double variation = transferTwiddleSizeDuration * currentInitialTransferDuration;
      double modifiedTransferDurationAlpha = (currentInitialTransferDuration + variation) / (currentTransferDuration + variation);

      applyTransferVariation(stepNumber, currentTransferDuration + variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentInitialTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end transfer duration gradient
      variation = transferTwiddleSizeDuration * currentEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (currentEndTransferDuration + variation) / (currentTransferDuration + variation);

      applyTransferVariation(stepNumber, currentTransferDuration + variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentEndTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applyTransferVariation(stepNumber, currentTransferDuration, currentTransferDurationAlpha, adjustedCoMPosition);
   }

   private void computeNextTransferGradient()
   {
      int stepNumber = 1;

      double nextTransferDuration = icpPlanner.getTransferDuration(stepNumber);
      double nextTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(stepNumber);

      double nextInitialTransferDuration = nextTransferDurationAlpha * nextTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferDurationAlpha) * nextTransferDuration;

      // compute initial transfer duration gradient
      double variation = transferTwiddleSizeDuration * nextInitialTransferDuration;
      double modifiedTransferDurationAlpha = (nextInitialTransferDuration + variation) / (nextTransferDuration + variation);

      applyTransferVariation(stepNumber, nextTransferDuration + variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextInitialTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end transfer duration gradient
      variation = transferTwiddleSizeDuration * nextEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (nextEndTransferDuration + variation) / (nextTransferDuration + variation);

      applyTransferVariation(stepNumber, nextTransferDuration + variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextEndTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applyTransferVariation(stepNumber, nextTransferDuration, nextTransferDurationAlpha, adjustedCoMPosition);

      if (nextInitialTransferGradient.containsNaN() || nextEndTransferGradient.containsNaN())
         throw new RuntimeException("Next Transfer Gradients Contains NaN.");
   }

   private void computeHigherTransferGradient(int stepIndex)
   {
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == stepIndex;

      double variation = transferTwiddleSizeDuration * icpPlanner.getTransferDuration(stepIndex);
      double originalDuration = icpPlanner.getTransferDuration(stepIndex);

      applyTransferVariation(isThisTheFinalTransfer, stepIndex, originalDuration + variation, adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      higherTransferGradients.get(stepIndex - 2).setByProjectionOntoXYPlane(tempGradient);

      // reinitialize
      applyTransferVariation(isThisTheFinalTransfer, stepIndex, originalDuration, adjustedCoMPosition);
   }

   private void applyTransferVariation(int stepNumber, double duration, double alpha, FramePoint2d comToPack)
   {
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == stepNumber;

      if (isThisTheFinalTransfer)
         icpPlanner.setFinalTransferDurationAlpha(alpha);
      else
         icpPlanner.setTransferDurationAlpha(stepNumber, alpha);

      applyTransferVariation(isThisTheFinalTransfer, stepNumber, duration, comToPack);
   }

   private void applyTransferVariation(boolean isThisTheFinalTransfer, int stepNumber, double duration, FramePoint2d comToPack)
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      if (isThisTheFinalTransfer)
         icpPlanner.setFinalTransferDuration(duration);
      else
         icpPlanner.setTransferDuration(stepNumber, duration);

      if (isInTransfer)
         icpPlanner.initializeForTransfer(currentInitialTime);
      else
         icpPlanner.initializeForSingleSupport(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }

   private void computeCurrentSwingGradient()
   {
      int stepNumber = 0;

      double currentSwingDuration = icpPlanner.getSwingDuration(stepNumber);
      double currentSwingDurationAlpha = icpPlanner.getSwingDurationAlpha(stepNumber);

      double currentInitialSwingDuration = currentSwingDurationAlpha * currentSwingDuration;
      double currentEndSwingDuration = (1.0 - currentSwingDurationAlpha) * currentSwingDuration;

      // compute initial swing duration gradient
      double currentDuration = icpPlanner.getSwingDuration(stepNumber);
      double variation = swingTwiddleSizeDuration * currentInitialSwingDuration;
      double modifiedSwingDurationAlpha = (currentInitialSwingDuration + variation) / (currentSwingDuration + variation);

      applySwingVariation(stepNumber, currentDuration + variation, modifiedSwingDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentInitialSwingGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end swing duration gradient
      icpPlanner.setSwingDuration(stepNumber, currentSwingDuration);
      variation = swingTwiddleSizeDuration * currentEndSwingDuration;
      modifiedSwingDurationAlpha = 1.0 - (currentEndSwingDuration + variation) / (currentSwingDuration + variation);

      applySwingVariation(stepNumber, currentDuration + variation, modifiedSwingDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentEndSwingGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applySwingVariation(stepNumber, currentDuration, currentSwingDurationAlpha, adjustedCoMPosition);
   }

   private void computeHigherSwingGradient(int stepIndex)
   {
      double duration = icpPlanner.getSwingDuration(stepIndex);
      double variation = swingTwiddleSizeDuration * duration;

      applySwingVariation(stepIndex, duration + variation, adjustedCoMPosition);

      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      higherSwingGradients.get(stepIndex - 1).setByProjectionOntoXYPlane(tempGradient);

      applySwingVariation(stepIndex, duration, adjustedCoMPosition);
   }

   private void applySwingVariation(int stepNumber, double duration, double alpha, FramePoint2d comToPack)
   {
      icpPlanner.setSwingDurationAlpha(stepNumber, alpha);

      applySwingVariation(stepNumber, duration, comToPack);
   }

   private void applySwingVariation(int stepNumber, double duration, FramePoint2d comToPack)
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      icpPlanner.setSwingDuration(stepNumber, duration);

      if (isInTransfer)
         icpPlanner.initializeForTransfer(currentInitialTime);
      else
         icpPlanner.initializeForSingleSupport(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }

   private void computeGradient(FramePoint originalPosition, FramePoint2d adjustedPosition, double variation, FrameVector gradientToPack)
   {
      originalPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(originalPosition);
      tempPoint.setZ(0.0);
      gradientToPack.setToZero(worldFrame);
      gradientToPack.setXY(adjustedPosition);
      gradientToPack.sub(tempPoint);
      gradientToPack.scale(1.0 / variation);
   }

   private void extractGradient(FrameVector2d gradientToExtract, RobotSide stanceSide, FrameVector gradientToPack)
   {
      gradientToPack.setToZero(worldFrame);
      gradientToPack.setXY(gradientToExtract);
      gradientToPack.changeFrame(stepDirectionFrames.get(stanceSide));
   }




   private static class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector xAxis)
      {
         xAxis.changeFrame(parentFrame);
         this.xAxis.setByProjectionOntoXYPlane(xAxis);
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
