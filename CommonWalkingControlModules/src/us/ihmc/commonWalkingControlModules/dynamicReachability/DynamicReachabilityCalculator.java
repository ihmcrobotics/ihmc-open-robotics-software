package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverInterface;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
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
import us.ihmc.robotics.linearAlgebra.MatrixTools;
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

   private static final boolean VISUALIZE = true;
   private static final double TRANSFER_TWIDDLE_SIZE = 0.2;
   private static final double SWING_TWIDDLE_SIZE = 0.1;

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

   private final IntegerYoVariable numberOfIterations = new IntegerYoVariable("numberOfIterations", registry);
   private final IntegerYoVariable maximumNumberOfIterations = new IntegerYoVariable("maximumNumberOfIterations", registry);

   private final DoubleYoVariable minimumTransferDuration = new DoubleYoVariable("minimumTransferDuration", registry);
   private final DoubleYoVariable maximumTransferDuration = new DoubleYoVariable("maximumTransferDuration", registry);
   private final DoubleYoVariable minimumSwingDuration = new DoubleYoVariable("minimumSwingDuration", registry);
   private final DoubleYoVariable maximumSwingDuration = new DoubleYoVariable("maximumSwingDuration", registry);

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

   private final ArrayList<YoFrameVector2d> yoTransferGradients = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> yoSwingGradients = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> transferTotalDurationAdjustments = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> swingTotalDurationAdjustments = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> transferDurationAdjustments = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> swingDurationAdjustments = new ArrayList<>();
   private final DoubleYoVariable requiredAdjustment = new DoubleYoVariable("requiredAdjustment", registry);
   private final YoFrameVector2d adjustmentSolution = new YoFrameVector2d("adjustmentSolution", worldFrame, registry);

   private final FrameVector transferGradient = new FrameVector();
   private final FrameVector swingGradient = new FrameVector();


   private final FramePoint tempPoint = new FramePoint();
   private final FramePoint2d tempFinalCoM = new FramePoint2d();
   private final FramePoint2d tempPoint2d = new FramePoint2d();

   private final FrameVector tempVector = new FrameVector();
   private final FrameVector tempHipVector = new FrameVector();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   private final double thighLength;
   private final double shinLength;

   private final SimpleActiveSetQPSolverInterface activeSetSolver = new SimpleEfficientActiveSetQPSolver();

   private static final double perpendicularWeight = 0.1;
   private static final double swingAdjustmentWeight = 1.0;
   private static final double transferAdjustmentWeight = 5.0;
   private static final double constraintWeight = 1000.0;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_h;

   private final DenseMatrix64F adjustmentObjective_H;
   private final DenseMatrix64F perpendicularObjective_H;
   private final DenseMatrix64F parallelObjective_H;
   private final DenseMatrix64F parallelObjective_h;

   private final DenseMatrix64F parallel_J;
   private final DenseMatrix64F perpendicular_J;

   private final DenseMatrix64F solverInput_Lb;
   private final DenseMatrix64F solverInput_Ub;

   private final DenseMatrix64F solution;

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      //maximumDesiredKneeBend.set(0.3);
      maximumDesiredKneeBend.set(0.2);
      maximumNumberOfIterations.set(3);

      minimumTransferDuration.set(0.15);
      maximumTransferDuration.set(5.0);
      minimumSwingDuration.set(0.4);
      maximumSwingDuration.set(10.0);

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
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         YoFrameVector2d yoTransferGradient = new YoFrameVector2d("transferGradient" + i, worldFrame, registry);
         YoFrameVector2d yoSwingGradient = new YoFrameVector2d("swingGradient" + i, worldFrame, registry);
         yoTransferGradients.add(yoTransferGradient);
         yoSwingGradients.add(yoSwingGradient);

         DoubleYoVariable transferTotalDurationAdjustment = new DoubleYoVariable("transferTotalDurationAdjustment" + i, registry);
         DoubleYoVariable swingTotalDurationAdjustment = new DoubleYoVariable("swingTotalDurationAdjustment" + i, registry);
         DoubleYoVariable transferDurationAdjustment = new DoubleYoVariable("transferDurationAdjustment" + i, registry);
         DoubleYoVariable swingDurationAdjustment = new DoubleYoVariable("swingDurationAdjustment" + i, registry);
         transferTotalDurationAdjustments.add(transferTotalDurationAdjustment);
         swingTotalDurationAdjustments.add(swingTotalDurationAdjustment);
         transferDurationAdjustments.add(transferDurationAdjustment);
         swingDurationAdjustments.add(swingDurationAdjustment);
      }

      solverInput_H = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
      solverInput_h = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

      solverInput_Lb = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);
      solverInput_Ub = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

      solution = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

      adjustmentObjective_H = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
      perpendicularObjective_H = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
      parallelObjective_H = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
      parallelObjective_h = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);
      parallel_J = new DenseMatrix64F(1, 2 * numberOfFootstepsToConsider);
      perpendicular_J = new DenseMatrix64F(1, 2 * numberOfFootstepsToConsider);


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

   /**
    * Sets the location of the next footstep in the plan
    * @param nextFootstep
    */
   public void setUpcomingFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }
   
   //// TODO: 3/22/17  this needs to work in single support too 

   /**
    * Checks whether the current footstep is reachable given the desired footstep timing. If it is, does nothing. If it is not, modifies the
    * ICP Plan timing to make sure that is is.
    */
   public void verifyAndEnsureReachability()
   {
      numberOfIterations.set(0);
      for (int i = 0; i < transferTotalDurationAdjustments.size(); i ++)
      {
         transferTotalDurationAdjustments.get(i).set(0.0);
         swingTotalDurationAdjustments.get(i).set(0.0);
         transferDurationAdjustments.get(i).set(0.0);
         swingDurationAdjustments.get(i).set(0.0);
      }

      boolean isStepReachable = checkReachabilityOfStep();

      while (!isStepReachable)
      {
         boolean needToMoveCoMBackward = (stanceHeightLine.getMaxPoint() <= stepHeightLine.getMinPoint());
         double requiredAdjustment;
         if (needToMoveCoMBackward)
            requiredAdjustment = computeRequiredAdjustmentBackward();
         else
            requiredAdjustment = computeRequiredAdjustmentForward();
         this.requiredAdjustment.set(requiredAdjustment);

         computeTransferGradient();
         computeSwingGradient();

         computeTimingAdjustment(requiredAdjustment);

         double initialTime = icpPlanner.getInitialTime();

         for (int i = 0; i < icpPlanner.getNumberOfFootstepsRegistered(); i++)
         {
            double transferDurationAdjustment = transferDurationAdjustments.get(i).getDoubleValue();
            double swingDurationAdjustment = swingDurationAdjustments.get(i).getDoubleValue();
            icpPlanner.adjustTransferDuration(i, transferDurationAdjustment);
            icpPlanner.adjustSwingDuration(i, swingDurationAdjustment);

            transferTotalDurationAdjustments.get(i).add(transferDurationAdjustment);
            swingTotalDurationAdjustments.get(i).add(swingDurationAdjustment);
         }
         icpPlanner.initializeForTransfer(initialTime);


         isStepReachable = checkReachabilityOfStep();
         numberOfIterations.increment();

         //if (numberOfIterations.getIntegerValue() > maximumNumberOfIterations.getIntegerValue() || requiredAdjustment == 0.0)
         if (numberOfIterations.getIntegerValue() > maximumNumberOfIterations.getIntegerValue())
         {
            break;
          }
      }
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


   private final double requiredAdjustmentSafetyFactor = 1.0;

   private double computeRequiredAdjustmentForward()
   {
      RobotSide stepSide = nextFootstep.getRobotSide();
      RobotSide stanceSide = stepSide.getOppositeSide();

      ReferenceFrame stanceHipFrame = predictedHipFrames.get(stanceSide);
      Vector2dZUpFrame stepDirectionFrame = stepDirectionFrames.get(stanceSide);

      // compute base point of upcoming sphere account for hip offsets
      FramePoint upcomingAnklePoint = ankleLocations.get(stanceSide.getOppositeSide());
      FramePoint stanceAnklePoint = ankleLocations.get(stanceSide);
      upcomingAnklePoint.changeFrame(stanceAnklePoint.getReferenceFrame());
      tempPoint.setToZero(predictedHipFrames.get(stepSide));
      tempPoint.changeFrame(stanceHipFrame);
      tempHipVector.setIncludingFrame(tempPoint);

      // compute step direction frame accounting for hip offsets
      tempPoint.setIncludingFrame(upcomingAnklePoint);
      tempPoint.changeFrame(stanceHipFrame);
      tempPoint.sub(tempHipVector);
      tempPoint.changeFrame(worldFrame);
      tempVector.setIncludingFrame(tempPoint);
      tempVector.sub(stanceAnklePoint);
      stepDirectionFrame.setXAxis(tempVector);

      // compute the actual planar step direction
      tempVector.changeFrame(stepDirectionFrame);
      double stepHeight = tempVector.getZ();
      double stepDistance = tempVector.getX();

      //// TODO: 3/23/17  make this account for stepping up / stepping down
      double minimumHipPosition = SphereIntersectionTools.computeMinimumDistanceToIntersectingPlane(stepDistance, stepHeight, minimumLegLength.getDoubleValue(),
            maximumLegLength.getDoubleValue());
      double maximumHipPosition = SphereIntersectionTools.computeMaximumDistanceToIntersectingPlane(stepDistance, stepHeight, maximumLegLength.getDoubleValue(),
            minimumLegLength.getDoubleValue());

      tempPoint.setToZero(stanceHipFrame);
      tempPoint.changeFrame(stepDirectionFrame);

      return requiredAdjustmentSafetyFactor * (minimumHipPosition - tempPoint.getX());
   }

   private double computeRequiredAdjustmentBackward()
   {
      // // FIXME: 3/22/17  this is differing from the reachability. Need to be using the hip points, not the center of mass

      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();
      FramePoint upcomingAnklePoint = ankleLocations.get(stanceSide.getOppositeSide());
      FramePoint stanceAnklePoint = ankleLocations.get(stanceSide);
      upcomingAnklePoint.changeFrame(stanceAnklePoint.getReferenceFrame());

      tempVector.setIncludingFrame(upcomingAnklePoint);
      tempVector.sub(stanceAnklePoint);
      tempVector2d.setByProjectionOntoXYPlaneIncludingFrame(tempVector);

      double stepDistance = tempVector2d.length();
      double minimumCoMPosition = (Math.pow(stepDistance, 2.0) - Math.pow(maximumLegLength.getDoubleValue(), 2.0) + Math.pow(minimumLegLength.getDoubleValue(), 2.0)) / (2.0 * stepDistance);
      double maximumCoMPosition = (Math.pow(stepDistance, 2.0) - Math.pow(minimumLegLength.getDoubleValue(), 2.0) + Math.pow(maximumLegLength.getDoubleValue(), 2.0)) / (2.0 * stepDistance);

      predictedCoMPosition.changeFrame(stepDirectionFrames.get(stanceSide));
      if (predictedCoMPosition.getX() < minimumCoMPosition)
      {
         return requiredAdjustmentSafetyFactor * (minimumCoMPosition - predictedCoMPosition.getX());
      }
      else if (predictedCoMPosition.getX() > maximumCoMPosition)
      {
         return requiredAdjustmentSafetyFactor * (maximumCoMPosition - predictedCoMPosition.getX());
      }

      return 0.0;
   }

   private double computeRequiredAdjustment()
   {
      // // FIXME: 3/22/17  this is differing from the reachability. Need to be using the hip points, not the center of mass

      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();
      FramePoint upcomingAnklePoint = ankleLocations.get(stanceSide.getOppositeSide());
      FramePoint stanceAnklePoint = ankleLocations.get(stanceSide);
      upcomingAnklePoint.changeFrame(stanceAnklePoint.getReferenceFrame());

      tempVector.setIncludingFrame(upcomingAnklePoint);
      tempVector.sub(stanceAnklePoint);
      tempVector2d.setByProjectionOntoXYPlaneIncludingFrame(tempVector);

      double stepDistance = tempVector2d.length();
      double minimumCoMPosition = (Math.pow(stepDistance, 2.0) - Math.pow(maximumLegLength.getDoubleValue(), 2.0) + Math.pow(minimumLegLength.getDoubleValue(), 2.0)) / (2.0 * stepDistance);
      double maximumCoMPosition = (Math.pow(stepDistance, 2.0) - Math.pow(minimumLegLength.getDoubleValue(), 2.0) + Math.pow(maximumLegLength.getDoubleValue(), 2.0)) / (2.0 * stepDistance);

      predictedCoMPosition.changeFrame(stepDirectionFrames.get(stanceSide));
      if (predictedCoMPosition.getX() < minimumCoMPosition)
      {
         return requiredAdjustmentSafetyFactor * (minimumCoMPosition - predictedCoMPosition.getX());
      }
      else if (predictedCoMPosition.getX() > maximumCoMPosition)
      {
         return requiredAdjustmentSafetyFactor * (maximumCoMPosition - predictedCoMPosition.getX());
      }

      return 0.0;
   }



   private void computeTimingAdjustment(double requiredAdjustment)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();
      transferGradient.changeFrame(stepDirectionFrames.get(stanceSide));
      swingGradient.changeFrame(stepDirectionFrames.get(stanceSide));

      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      solution.reshape(2 * numberOfFootstepsRegistered, 2 * numberOfFootstepsRegistered);

      adjustmentObjective_H.reshape(2 * numberOfFootstepsRegistered, 2 * numberOfFootstepsRegistered);
      perpendicularObjective_H.reshape(2 * numberOfFootstepsRegistered, 2 * numberOfFootstepsRegistered);
      parallelObjective_H.reshape(2 * numberOfFootstepsRegistered, 2 * numberOfFootstepsRegistered);
      parallelObjective_h.reshape(2 * numberOfFootstepsRegistered, 1);

      parallel_J.reshape(1, 2 * numberOfFootstepsRegistered);
      perpendicular_J.reshape(1, 2 * numberOfFootstepsRegistered);

      solverInput_H.reshape(2 * numberOfFootstepsRegistered, 2 * numberOfFootstepsRegistered);
      solverInput_h.reshape(2 * numberOfFootstepsRegistered, 1);

      solverInput_Lb.reshape(2 * numberOfFootstepsRegistered, 1);
      solverInput_Ub.reshape(2 * numberOfFootstepsRegistered, 1);

      solution.zero();

      adjustmentObjective_H.zero();
      perpendicularObjective_H.zero();
      parallelObjective_H.zero();
      parallelObjective_h.zero();

      parallel_J.zero();
      perpendicular_J.zero();

      solverInput_H.zero();
      solverInput_h.zero();
      solverInput_Lb.zero();
      solverInput_Ub.zero();

      for (int stepIndex = 0; stepIndex < numberOfFootstepsRegistered; stepIndex++)
      {
         transferGradient.setToZero(worldFrame);
         swingGradient.setToZero(worldFrame);
         yoTransferGradients.get(stepIndex).getFrameTuple2d(tempVector2d);
         transferGradient.setXY(tempVector2d);
         yoSwingGradients.get(stepIndex).getFrameTuple2d(tempVector2d);
         swingGradient.setXY(tempVector2d);

         transferGradient.changeFrame(stepDirectionFrames.get(stanceSide));
         swingGradient.changeFrame(stepDirectionFrames.get(stanceSide));

         double transferPerpendicularGradient = transferGradient.getY();
         double transferParallelGradient = transferGradient.getX();
         double swingPerpendicularGradient = swingGradient.getY();
         double swingParallelGradient = swingGradient.getX();

         parallel_J.set(0, 2 * stepIndex, transferParallelGradient);
         parallel_J.set(0, 2 * stepIndex + 1, swingParallelGradient);
         perpendicular_J.set(0, 2 * stepIndex, transferPerpendicularGradient);
         perpendicular_J.set(0, 2 * stepIndex + 1, swingPerpendicularGradient);

         adjustmentObjective_H.set(2 * stepIndex, 2 * stepIndex, transferAdjustmentWeight);
         adjustmentObjective_H.set(2 * stepIndex + 1, 2 * stepIndex + 1, swingAdjustmentWeight);

         double transferDuration = icpPlanner.getTransferDuration(stepIndex);
         double swingDuration = icpPlanner.getSwingDuration(stepIndex);

         solverInput_Lb.set(2 * stepIndex, 0, minimumTransferDuration.getDoubleValue() - transferDuration);
         solverInput_Lb.set(2 * stepIndex + 1, 0, minimumSwingDuration.getDoubleValue() - swingDuration);
         solverInput_Ub.set(2 * stepIndex, 0, maximumTransferDuration.getDoubleValue() - transferDuration);
         solverInput_Ub.set(2 * stepIndex + 1, 0, maximumSwingDuration.getDoubleValue() - swingDuration);
      }

      CommonOps.multTransA(perpendicular_J, perpendicular_J, perpendicularObjective_H);
      CommonOps.scale(perpendicularWeight, perpendicularObjective_H);

      CommonOps.multTransA(parallel_J, parallel_J, parallelObjective_H);
      CommonOps.scale(constraintWeight, parallelObjective_H);

      CommonOps.transpose(parallel_J, parallelObjective_h);
      CommonOps.scale(-2.0 * requiredAdjustment * constraintWeight, parallelObjective_h);

      double scalar = Math.pow(requiredAdjustment, 2.0) * constraintWeight;

      
      
      CommonOps.add(solverInput_H, adjustmentObjective_H, solverInput_H);
      CommonOps.add(solverInput_H, perpendicularObjective_H, solverInput_H);
      CommonOps.add(solverInput_H, parallelObjective_H, solverInput_H);

      CommonOps.add(solverInput_h, parallelObjective_h, solverInput_h);


      activeSetSolver.clear();

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, scalar);
      activeSetSolver.setVariableBounds(solverInput_Lb, solverInput_Ub);
      int numberOfIterations = activeSetSolver.solve(solution);

      if (MatrixTools.containsNaN(solution))
      {
         NoConvergenceException e = new NoConvergenceException(numberOfIterations);
         e.printStackTrace();
         PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
      }

      for (int stepIndex = 0; stepIndex < numberOfFootstepsRegistered; stepIndex++)
      {
         transferDurationAdjustments.get(stepIndex).set(solution.get(2 * stepIndex, 0));
         swingDurationAdjustments.get(stepIndex).set(solution.get(2 * stepIndex + 1, 0));
      }
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


   private void computeTransferGradient()
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      int numberOfStepsRegister = icpPlanner.getNumberOfFootstepsRegistered();
      for (int stepIndex = 0; stepIndex < numberOfStepsRegister; stepIndex++)
      {
         double variation = TRANSFER_TWIDDLE_SIZE * icpPlanner.getTransferDuration(stepIndex);
         icpPlanner.adjustTransferDuration(stepIndex, variation);
         icpPlanner.initializeForTransfer(currentInitialTime);

         icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

         icpPlanner.adjustTransferDuration(stepIndex, -variation);
         icpPlanner.initializeForTransfer(currentInitialTime);

         predictedCoMPosition.changeFrame(worldFrame);
         tempPoint.setToZero(worldFrame);
         tempPoint.set(predictedCoMPosition);
         tempPoint.setZ(0.0);
         transferGradient.setToZero(worldFrame);
         transferGradient.setXY(adjustedCoMPosition);
         transferGradient.sub(tempPoint);
         transferGradient.scale(1.0 / variation);

         yoTransferGradients.get(stepIndex).setByProjectionOntoXYPlane(transferGradient);
      }
   }

   private void computeSwingGradient()
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      int numberOfStepsRegister = icpPlanner.getNumberOfFootstepsRegistered();
      for (int stepIndex = 0; stepIndex < numberOfStepsRegister; stepIndex++)
      {
         double variation = SWING_TWIDDLE_SIZE * icpPlanner.getSwingDuration(stepIndex);
         icpPlanner.adjustSwingDuration(stepIndex, variation);
         icpPlanner.initializeForTransfer(currentInitialTime);

         icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

         icpPlanner.adjustSwingDuration(stepIndex, -variation);
         icpPlanner.initializeForTransfer(currentInitialTime);

         predictedCoMPosition.changeFrame(worldFrame);
         tempPoint.setToZero(worldFrame);
         tempPoint.set(predictedCoMPosition);
         tempPoint.setZ(0.0);
         swingGradient.setToZero(worldFrame);
         swingGradient.setXY(adjustedCoMPosition);
         swingGradient.sub(tempPoint);
         swingGradient.scale(1.0 / variation);

         yoSwingGradients.get(stepIndex).setByProjectionOntoXYPlane(swingGradient);
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
