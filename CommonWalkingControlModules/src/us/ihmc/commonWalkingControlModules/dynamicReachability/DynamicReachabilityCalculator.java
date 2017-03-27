package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;

public class DynamicReachabilityCalculator
{
   //// TODO: 3/21/17 cleanup
   //// TODO: 3/21/17 disable checks on swing leg bending if the upcoming step is a step up
   //// TODO: 3/21/17 disable checks on stance leg bending if the upcoming step is a step down 
   //// TODO: 3/21/17 add in the ability to angle the hip forward for reachability
   //// TODO: 3/21/17 add in the ability to drop the pelvis for reachability
   //// TODO: 3/25/17 explore for step up and step down for sphere intersection

   private static final boolean VISUALIZE = true;
   private static final double TRANSFER_TWIDDLE_SIZE = 0.2;
   private static final double SWING_TWIDDLE_SIZE = 0.2;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable minimumLegLength = new DoubleYoVariable("minimumLegLength", registry);
   private final DoubleYoVariable maximumLegLength = new DoubleYoVariable("maximumLegLength", registry);

   private final DoubleYoVariable maximumDesiredKneeBend = new DoubleYoVariable("maximumDesiredKneeBend", registry);

   private final DoubleYoVariable stanceLegMinimumHeight = new DoubleYoVariable("stanceLegMinimumHeight", registry);
   private final DoubleYoVariable stanceLegMaximumHeight = new DoubleYoVariable("stanceLegMaximumHeight", registry);
   private final DoubleYoVariable swingLegMinimumHeight = new DoubleYoVariable("swingLegMinimumHeight", registry);
   private final DoubleYoVariable swingLegMaximumHeight = new DoubleYoVariable("swingLegMaximumHeight", registry);

   private final BooleanYoVariable reachableWRTStanceFoot = new BooleanYoVariable("reachableWRTStanceFoot", registry);
   private final BooleanYoVariable reachableWRTFootstep = new BooleanYoVariable("reachableWRTFootstep", registry);
   private final BooleanYoVariable isStepReachable = new BooleanYoVariable("isStepReachable", registry);

   private final IntegerYoVariable numberOfAdjustments = new IntegerYoVariable("numberOfAdjustments", registry);
   private final IntegerYoVariable numberOfForwardAdjustments = new IntegerYoVariable("numberOfForwardAdjustments", registry);
   private final IntegerYoVariable numberOfBackwardAdjustments = new IntegerYoVariable("numberOfBackwardAdjustments", registry);
   private final IntegerYoVariable maximumNumberOfAdjustments = new IntegerYoVariable("maximumNumberOfAdjustments", registry);

   private final SideDependentList<YoFramePoint> hipMinimumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> hipMaximumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> yoAnkleLocations = new SideDependentList<>();

   private final SideDependentList<FramePoint> ankleLocations = new SideDependentList<>();

   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private Footstep nextFootstep;

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

   private final YoFrameVector2d currentInitialTransferGradient = new YoFrameVector2d("currentInitialTransferGradient", worldFrame, registry);
   private final YoFrameVector2d currentEndTransferGradient = new YoFrameVector2d("currentEndTransferGradient", worldFrame, registry);

   private final YoFrameVector2d nextInitialTransferGradient = new YoFrameVector2d("nextInitialTransferGradient", worldFrame, registry);
   private final YoFrameVector2d nextEndTransferGradient = new YoFrameVector2d("nextEndTransferGradient", worldFrame, registry);

   private final YoFrameVector2d initialSwingGradient = new YoFrameVector2d("initialSwingGradient", worldFrame, registry);
   private final YoFrameVector2d endSwingGradient = new YoFrameVector2d("endSwingGradient", worldFrame, registry);

   private final ArrayList<YoFrameVector2d> higherTransferGradients = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> higherSwingGradients = new ArrayList<>();

   private final DoubleYoVariable currentTransferTotalAdjustment = new DoubleYoVariable("currentTransferTotalAdjustment", registry);
   private final DoubleYoVariable swingTotalAdjustment = new DoubleYoVariable("swingTotalAdjustment", registry);
   private final DoubleYoVariable nextTransferTotalAdjustment = new DoubleYoVariable("nextTransferTotalAdjustment", registry);
   private final DoubleYoVariable currentTransferAdjustment = new DoubleYoVariable("currentTransferAdjustment", registry);
   private final DoubleYoVariable swingAdjustment = new DoubleYoVariable("swingAdjustment", registry);
   private final DoubleYoVariable nextTransferAdjustment = new DoubleYoVariable("nextTransferAdjustment", registry);

   private final DoubleYoVariable currentTransferAlpha = new DoubleYoVariable("currentTransferAlpha", registry);
   private final DoubleYoVariable swingAlpha = new DoubleYoVariable("swingAlpha", registry);
   private final DoubleYoVariable nextTransferAlpha = new DoubleYoVariable("nextTransferAlpha", registry);

   private final DoubleYoVariable requiredAdjustment = new DoubleYoVariable("requiredAdjustment", registry);

   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("isInTransfer", registry);

   private final FrameVector tempGradient = new FrameVector();

   private final FramePoint tempPoint = new FramePoint();
   private final FramePoint2d tempPoint2d = new FramePoint2d();
   private final FramePoint2d tempFinalCoM = new FramePoint2d();

   private final FrameVector tempVector = new FrameVector();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   private final double thighLength;
   private final double shinLength;

   private static final double requiredAdjustmentSafetyFactor = 1.0;

   private final TimeAdjustmentSolver solver;

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      maximumDesiredKneeBend.set(0.2);
      maximumNumberOfAdjustments.set(3);

      solver = new TimeAdjustmentSolver(icpPlanner.getNumberOfFootstepsToConsider(), registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleLocations.put(robotSide, new FramePoint());

         YoFramePoint hipMaximumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMaximumPoint", worldFrame, registry);
         YoFramePoint hipMinimumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMinimumPoint", worldFrame, registry);
         hipMaximumLocations.put(robotSide, hipMaximumLocation);
         hipMinimumLocations.put(robotSide, hipMinimumLocation);

         YoFramePoint ankleLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "AnkleLocation", worldFrame, registry);
         yoAnkleLocations.put(robotSide, ankleLocation);
      }

      int numberOfFootstepsToConsider = icpPlanner.getNumberOfFootstepsToConsider();
      for (int i = 0; i < numberOfFootstepsToConsider - 3; i++)
      {
         YoFrameVector2d higherTransferGradient = new YoFrameVector2d("higherTransferGradient" + i, worldFrame, registry);
         YoFrameVector2d higherSwingGradient = new YoFrameVector2d("higherSwingGradient" + i, worldFrame, registry);
         higherTransferGradients.add(higherTransferGradient);
         higherSwingGradients.add(higherSwingGradient);
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
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      for (RobotSide side : RobotSide.values)
      {
         YoFramePoint hipMaximumLocation = hipMaximumLocations.get(side);
         YoFramePoint hipMinimumLocation = hipMinimumLocations.get(side);

         YoGraphicPosition hipMaximumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Maximum Hip Point", hipMaximumLocation, 0.05, YoAppearance.ForestGreen());
         YoGraphicPosition hipMinimumLocationViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Predicted Minimum Hip Point", hipMinimumLocation, 0.05, YoAppearance.Blue());

         AppearanceDefinition maxAppearance = YoAppearance.Green();
         AppearanceDefinition minAppearance = YoAppearance.Red();
         maxAppearance.setTransparency(0.8);
         minAppearance.setTransparency(0.8);
         YoFramePoint ankleLocation = yoAnkleLocations.get(side);
         YoGraphicPosition minimumReachabilityViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Minimum Reachability", ankleLocation, minimumLegLength.getDoubleValue(), minAppearance);
         YoGraphicPosition maximumReachabilityViz = new YoGraphicPosition(side.getSideNameFirstLetter() + "Maximum Reachability", ankleLocation, maximumLegLength.getDoubleValue(), maxAppearance);

         yoGraphicsList.add(hipMaximumLocationViz);
         yoGraphicsList.add(hipMinimumLocationViz);
         yoGraphicsList.add(minimumReachabilityViz);
         yoGraphicsList.add(maximumReachabilityViz);
      }

      yoGraphicsList.setVisible(VISUALIZE);
      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private void updateFrames(Footstep nextFootstep)
   {
      RobotSide swingSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      icpPlanner.getFinalDesiredCenterOfMassPosition(tempFinalCoM);
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
      yoAnkleLocations.get(stanceSide).set(stanceAnkleLocation);
      yoAnkleLocations.get(swingSide).set(upcomingStepLocation);

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

      double minimumLegLength = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) +
            2 * thighLength * shinLength * Math.cos(maximumDesiredKneeBend.getDoubleValue());
      minimumLegLength = Math.sqrt(minimumLegLength);
      this.minimumLegLength.set(minimumLegLength);
   }

   private void computeHeightLineFromStance(RobotSide supportSide)
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
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         maximumHeight += ankleLocation.getZ();
      }

      hipMaximumLocations.get(supportSide).setZ(maximumHeight);
      hipMinimumLocations.get(supportSide).setZ(minimumHeight);

      stanceLegMinimumHeight.set(minimumHeight);
      stanceLegMaximumHeight.set(maximumHeight);
      stanceHeightLine.set(minimumHeight, maximumHeight);
   }

   private void computeHeightLineFromStep(Footstep nextFootstep)
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
         minimumHeight = Math.sqrt(Math.pow(minimumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
         minimumHeight += ankleLocation.getZ();
      }
      if (planarDistance >= maximumLegLength.getDoubleValue())
      {
         maximumHeight = 0.0;
      }
      else
      {
         maximumHeight = Math.sqrt(Math.pow(maximumLegLength.getDoubleValue(), 2.0) - Math.pow(planarDistance, 2.0));
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
      numberOfForwardAdjustments.set(0);
      numberOfBackwardAdjustments.set(0);

      currentTransferTotalAdjustment.set(0.0);
      swingTotalAdjustment.set(0.0);
      nextTransferTotalAdjustment.set(0.0);
      currentTransferAdjustment.set(0.0);
      swingAdjustment.set(0.0);
      nextTransferAdjustment.set(0.0);
   }


   /**
    * Sets the location of the next footstep in the plan
    * @param nextFootstep
    */
   public void setUpcomingFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }
   
   /**
    * Checks whether the current footstep is reachable given the desired footstep timing. If it is, does nothing. If it is not, modifies the
    * ICP Plan timing to make sure that is is.
    */
   public void verifyAndEnsureReachability()
   {
      reset();

      boolean isStepReachable = checkReachabilityOfStep();

      while (!isStepReachable)
      {
         boolean needToMoveCoMBackward = (stanceHeightLine.getMaxPoint() <= stepHeightLine.getMinPoint());
         double requiredAdjustment = computeRequiredAdjustment(needToMoveCoMBackward);
         this.requiredAdjustment.set(requiredAdjustment);

         if (numberOfAdjustments.getIntegerValue() > maximumNumberOfAdjustments.getIntegerValue())
            throw new RuntimeException();

         computeCurrentTransferGradient();
         computeNextTransferGradient();
         computeSwingGradient();

         computeTimingAdjustment(requiredAdjustment);

         double initialTime = icpPlanner.getInitialTime();

         double currentTransferDurationAdjustment = currentTransferAdjustment.getDoubleValue();
         currentTransferTotalAdjustment.add(currentTransferDurationAdjustment);
         icpPlanner.adjustTransferDuration(0, currentTransferDurationAdjustment);
         icpPlanner.setTransferDurationAlpha(0, currentTransferAlpha.getDoubleValue());
         double swingDurationAdjustment = swingAdjustment.getDoubleValue();
         swingTotalAdjustment.add(swingDurationAdjustment);
         icpPlanner.adjustSwingDuration(0, swingDurationAdjustment);
         icpPlanner.setSwingDurationAlpha(0, swingAlpha.getDoubleValue());
         double nextTransferDurationAdjustment = nextTransferAdjustment.getDoubleValue();
         nextTransferTotalAdjustment.add(nextTransferDurationAdjustment);
         icpPlanner.adjustTransferDuration(1, nextTransferDurationAdjustment);
         icpPlanner.setTransferDurationAlpha(1, nextTransferAlpha.getDoubleValue());

         if (isInTransfer.getBooleanValue())
            icpPlanner.initializeForTransfer(initialTime);
         else
            icpPlanner.initializeForSingleSupport(initialTime);

         isStepReachable = checkReachabilityOfStep();
         numberOfAdjustments.increment();
      }
   }

   public void setInTransfer()
   {
      isInTransfer.set(true);
   }

   public void setInSwing()
   {
      isInTransfer.set(false);
   }

   /**
    * Checks whether the current footstep is reachable given the desired footstep timing.
    *
    * @return reachable or not
    */
   public boolean checkReachabilityOfStep()
   {
      RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

      updateFrames(nextFootstep);
      updateLegLengthLimits();

      computeHeightLineFromStance(supportSide);
      computeHeightLineFromStep(nextFootstep);

      this.reachableWRTStanceFoot.set(stanceHeightLine.length() > 0.0);
      this.reachableWRTFootstep.set(stepHeightLine.length() > 0.0);
      this.isStepReachable.set(stanceHeightLine.isOverlappingExclusive(stepHeightLine));

      return isStepReachable.getBooleanValue();
   }


   private final FramePoint adjustedUpcomingAnklePoint = new FramePoint();
   private final FramePoint adjustedStanceAnklePoint = new FramePoint();
   private final FrameVector stepHipVector = new FrameVector();
   private final FrameVector stanceHipVector = new FrameVector();

   private double computeRequiredAdjustment(boolean needToMoveCoMBackward)
   {
      RobotSide stepSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = stepSide.getOppositeSide();

      ReferenceFrame stanceHipFrame = predictedHipFrames.get(stanceSide);
      ReferenceFrame stepHipFrame = predictedHipFrames.get(stepSide);
      Vector2dZUpFrame stepDirectionFrame = stepDirectionFrames.get(stanceSide);

      // compute base point of upcoming sphere account for hip offsets
      FramePoint upcomingAnklePoint = ankleLocations.get(stanceSide.getOppositeSide());
      FramePoint stanceAnklePoint = ankleLocations.get(stanceSide);

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

      double minimumHipPosition = SphereIntersectionTools.computeMinimumDistanceToIntersectingPlane(stepDistance, stepHeight, minimumLegLength.getDoubleValue(),
            maximumLegLength.getDoubleValue());
      double maximumHipPosition = SphereIntersectionTools.computeMinimumDistanceToIntersectingPlane(stepDistance, stepHeight, maximumLegLength.getDoubleValue(),
            minimumLegLength.getDoubleValue());

      tempPoint.setToZero(predictedCoMFrame);
      tempPoint.changeFrame(worldFrame);
      tempPoint.sub(adjustedStanceAnklePoint);
      tempPoint.changeFrame(stepDirectionFrame);

      if (needToMoveCoMBackward)
      {
         numberOfBackwardAdjustments.increment();

         return requiredAdjustmentSafetyFactor * (maximumHipPosition - tempPoint.getX());
      }
      else
      {
         numberOfForwardAdjustments.increment();

         return requiredAdjustmentSafetyFactor * (minimumHipPosition - tempPoint.getX());
      }
   }

   private void computeTimingAdjustment(double requiredAdjustment)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();

      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      solver.setNumberOfFootstepsToConsider(icpPlanner.getNumberOfFootstepsToConsider());
      solver.setNumberOfFootstepsRegistered(numberOfFootstepsRegistered);

      solver.reshape();

      //reshape();

      extractGradient(currentInitialTransferGradient, stanceSide, tempGradient);
      solver.setCurrentInitialTransferGradient(tempGradient);

      extractGradient(currentEndTransferGradient, stanceSide, tempGradient);
      solver.setCurrentEndTransferGradient(tempGradient);

      extractGradient(initialSwingGradient, stanceSide, tempGradient);
      solver.setCurrentInitialSwingGradient(tempGradient);

      extractGradient(endSwingGradient, stanceSide, tempGradient);
      solver.setCurrentEndSwingGradient(tempGradient);

      extractGradient(nextInitialTransferGradient, stanceSide, tempGradient);
      solver.setNextInitialTransferGradient(tempGradient);

      extractGradient(nextEndTransferGradient, stanceSide, tempGradient);
      solver.setNextEndTransferGradient(tempGradient);

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

      try
      {
         solver.compute();
      }
      catch (NoConvergenceException e)
      {
         e.printStackTrace();
         PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
      }

      extractSolution();
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

   private void extractGradient(YoFrameVector2d gradientToExtract, RobotSide stanceSide, FrameVector gradientToPack)
   {
      gradientToPack.setToZero(worldFrame);
      gradientToExtract.getFrameTuple2d(tempVector2d);
      gradientToPack.setXY(tempVector2d);
      gradientToPack.changeFrame(stepDirectionFrames.get(stanceSide));
   }

   private void extractSolution()
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
      currentTransferAdjustment.set(currentInitialTransferAdjustment + currentEndTransferAdjustment);
      this.currentTransferAlpha.set(currentInitialTransferDuration / (currentInitialTransferDuration + currentEndTransferDuration));

      // handle current swing
      double initialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double endSwingAdjustment = solver.getCurrentEndSwingAdjustment();

      double swingDuration = icpPlanner.getSwingDuration(0);
      double swingAlpha = icpPlanner.getSwingDurationAlpha(0);
      double swingInitialDuration = swingAlpha * swingDuration;
      double swingEndDuration = (1.0 - swingAlpha) * swingDuration;

      swingInitialDuration += initialSwingAdjustment;
      swingEndDuration += endSwingAdjustment;
      swingAdjustment.set(initialSwingAdjustment + endSwingAdjustment);
      this.swingAlpha.set(swingInitialDuration / (swingInitialDuration + swingEndDuration));

      // handle next transfer
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      double nextTransferAlpha = icpPlanner.getTransferDurationAlpha(1);
      double nextInitialTransferDuration = nextTransferAlpha * currentTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferAlpha) * nextTransferDuration;

      nextInitialTransferDuration += nextInitialTransferAdjustment;
      nextEndTransferDuration += nextEndTransferAdjustment;
      nextTransferAdjustment.set(nextInitialTransferAdjustment + nextEndTransferAdjustment);
      this.nextTransferAlpha.set(nextInitialTransferDuration / (nextInitialTransferDuration + nextEndTransferDuration));
   }



   private void computeCurrentTransferGradient()
   {
      int stepNumber = 0;

      double currentTransferDuration = icpPlanner.getTransferDuration(stepNumber);
      double currentTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(stepNumber);

      double currentInitialTransferDuration = currentTransferDurationAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferDurationAlpha) * currentTransferDuration;

      // compute initial transfer duration gradient
      double variation = TRANSFER_TWIDDLE_SIZE * currentInitialTransferDuration;
      double modifiedTransferDurationAlpha = (currentInitialTransferDuration + variation) / (currentTransferDuration + variation);

      applyTransferVariation(stepNumber, currentTransferDuration, variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentInitialTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end transfer duration gradient
      icpPlanner.adjustTransferDuration(stepNumber, -variation);
      variation = TRANSFER_TWIDDLE_SIZE * currentEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (currentEndTransferDuration + variation) / (currentTransferDuration + variation);

      applyTransferVariation(stepNumber, currentTransferDuration, variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      currentEndTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applyTransferVariation(stepNumber, currentTransferDuration, -variation, currentTransferDurationAlpha, adjustedCoMPosition);
   }

   private void computeNextTransferGradient()
   {
      int stepNumber = 1;
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == stepNumber;

      double nextTransferDuration = icpPlanner.getTransferDuration(stepNumber);
      double nextTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(stepNumber);

      double nextInitialTransferDuration = nextTransferDurationAlpha * nextTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferDurationAlpha) * nextTransferDuration;

      // compute initial transfer duration gradient
      double variation = TRANSFER_TWIDDLE_SIZE * nextInitialTransferDuration;
      double modifiedTransferDurationAlpha = (nextInitialTransferDuration + variation) / (nextTransferDuration + variation);

      applyTransferVariation(stepNumber, nextTransferDuration, variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextInitialTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end transfer duration gradient
      if (isThisTheFinalTransfer)
         icpPlanner.setFinalTransferDuration(nextTransferDuration);
      else
         icpPlanner.adjustTransferDuration(stepNumber, -variation);

      variation = TRANSFER_TWIDDLE_SIZE * nextEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (nextEndTransferDuration + variation) / (nextTransferDuration + variation);

      applyTransferVariation(stepNumber, nextTransferDuration, variation, modifiedTransferDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      nextEndTransferGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applyTransferVariation(stepNumber, nextTransferDuration, -variation, nextTransferDurationAlpha, adjustedCoMPosition);
   }

   private void applyTransferVariation(int stepNumber, double originalDuration, double timeVariation, double alpha, FramePoint2d comToPack)
   {
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == stepNumber;

      double currentInitialTime = icpPlanner.getInitialTime();

      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(originalDuration + timeVariation);
         icpPlanner.setFinalTransferDurationAlpha(alpha);
      }
      else
      {
         icpPlanner.adjustTransferDuration(stepNumber, timeVariation);
         icpPlanner.setTransferDurationAlpha(stepNumber, alpha);
      }

      if (isInTransfer.getBooleanValue())
         icpPlanner.initializeForTransfer(currentInitialTime);
      else
         icpPlanner.initializeForSingleSupport(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }

   private void computeSwingGradient()
   {
      int stepNumber = 0;

      double currentSwingDuration = icpPlanner.getSwingDuration(stepNumber);
      double currentSwingDurationAlpha = icpPlanner.getSwingDurationAlpha(stepNumber);

      double currentInitialSwingDuration = currentSwingDurationAlpha * currentSwingDuration;
      double currentEndSwingDuration = (1.0 - currentSwingDurationAlpha) * currentSwingDuration;

      // compute initial swing duration gradient
      double variation = SWING_TWIDDLE_SIZE * currentInitialSwingDuration;
      double modifiedSwingDurationAlpha = (currentInitialSwingDuration + variation) / (currentSwingDuration + variation);

      applySwingVariation(stepNumber, variation, modifiedSwingDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      initialSwingGradient.setByProjectionOntoXYPlane(tempGradient);

      // compute end swing duration gradient
      icpPlanner.adjustSwingDuration(stepNumber, -variation);
      variation = SWING_TWIDDLE_SIZE * currentEndSwingDuration;
      modifiedSwingDurationAlpha = 1.0 - (currentEndSwingDuration + variation) / (currentSwingDuration + variation);

      applySwingVariation(stepNumber, variation, modifiedSwingDurationAlpha, adjustedCoMPosition);
      computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
      endSwingGradient.setByProjectionOntoXYPlane(tempGradient);

      // reset everything to normal
      applySwingVariation(stepNumber, -variation, currentSwingDurationAlpha, adjustedCoMPosition);
   }

   private void applySwingVariation(int stepNumber, double timeVariation, double alpha, FramePoint2d comToPack)
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      icpPlanner.adjustSwingDuration(stepNumber, timeVariation);
      icpPlanner.setSwingDurationAlpha(stepNumber, alpha);

      if (isInTransfer.getBooleanValue())
         icpPlanner.initializeForTransfer(currentInitialTime);
      else
         icpPlanner.initializeForSingleSupport(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(comToPack);
   }


   //// TODO: 3/24/17  
   private void computeHigherTransferGradients()
   {
      boolean isInTransfer = this.isInTransfer.getBooleanValue();
      double currentInitialTime = icpPlanner.getInitialTime();

     int numberOfStepsRegister = icpPlanner.getNumberOfFootstepsRegistered();
      for (int stepIndex = 0; stepIndex < numberOfStepsRegister; stepIndex++)
      {
         double variation = TRANSFER_TWIDDLE_SIZE * icpPlanner.getTransferDuration(stepIndex);
         icpPlanner.adjustTransferDuration(stepIndex, variation);
         if (isInTransfer)
            icpPlanner.initializeForTransfer(currentInitialTime);
         else
            icpPlanner.initializeForSingleSupport(currentInitialTime);

         icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

         icpPlanner.adjustTransferDuration(stepIndex, -variation);
         if (isInTransfer)
            icpPlanner.initializeForTransfer(currentInitialTime);
         else
            icpPlanner.initializeForSingleSupport(currentInitialTime);

         computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
         higherTransferGradients.get(stepIndex).setByProjectionOntoXYPlane(tempGradient);
      }
   }

   //// TODO: 3/24/17  
   private void computeHigherSwingGradients()
   {
      boolean isInTransfer = this.isInTransfer.getBooleanValue();
      double currentInitialTime = icpPlanner.getInitialTime();

      int numberOfStepsRegister = icpPlanner.getNumberOfFootstepsRegistered();
      for (int stepIndex = 0; stepIndex < numberOfStepsRegister; stepIndex++)
      {
         double variation = SWING_TWIDDLE_SIZE * icpPlanner.getSwingDuration(stepIndex);
         icpPlanner.adjustSwingDuration(stepIndex, variation);
         if (isInTransfer)
            icpPlanner.initializeForTransfer(currentInitialTime);
         else
            icpPlanner.initializeForSingleSupport(currentInitialTime);

         icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

         icpPlanner.adjustSwingDuration(stepIndex, -variation);
         if (isInTransfer)
            icpPlanner.initializeForTransfer(currentInitialTime);
         else
            icpPlanner.initializeForSingleSupport(currentInitialTime);

         computeGradient(predictedCoMPosition, adjustedCoMPosition, variation, tempGradient);
         higherSwingGradients.get(stepIndex).setByProjectionOntoXYPlane(tempGradient);
      }
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

      public void setXAxis(FrameVector2d xAxis)
      {
         xAxis.changeFrame(parentFrame);
         this.xAxis.set(xAxis);
         this.xAxis.normalize();
         update();
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
