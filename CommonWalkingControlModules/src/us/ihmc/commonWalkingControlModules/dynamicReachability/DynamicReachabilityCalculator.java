package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverInterface;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
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

public class DynamicReachabilityCalculator
{
   //// TODO: 3/21/17 cleanup
   //// TODO: 3/21/17 disable checks on swing leg bending if the upcoming step is a step up
   //// TODO: 3/21/17 disable checks on stance leg bending if the upcoming step is a step down 
   //// TODO: 3/21/17 add in the ability to angle the hip forward for reachability
   //// TODO: 3/21/17 add in the ability to drop the pelvis for reachability

   private static final boolean VISUALIZE = true;
   private static final double TWIDDLE_SIZE = 0.05;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable minimumLegLength = new DoubleYoVariable("minimumLegLength", registry);
   private final DoubleYoVariable maximumLegLength = new DoubleYoVariable("maximumLegLength", registry);

   private final DoubleYoVariable transferTimeAdjustment = new DoubleYoVariable("transferTimeAdjustment", registry);
   private final DoubleYoVariable swingTimeAdjustment = new DoubleYoVariable("swingTimeAdjustment", registry);

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

   private final SideDependentList<YoFramePoint> hipMinimumLocations = new SideDependentList<>();
   private final SideDependentList<YoFramePoint> hipMaximumLocations = new SideDependentList<>();

   private final SideDependentList<FramePoint> ankleLocations = new SideDependentList<>();

   private final ICPPlanner icpPlanner;
   private final FullHumanoidRobotModel fullRobotModel;

   private Footstep nextFootstep;

   private final double thighLength;
   private final double shinLength;

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

   private final YoFrameVector2d yoTransferGradient = new YoFrameVector2d("transferGradient", worldFrame, registry);
   private final YoFrameVector2d yoSwingGradient = new YoFrameVector2d("swingGradient", worldFrame, registry);
   private final DoubleYoVariable requiredAdjustment = new DoubleYoVariable("requiredAdjustment", registry);
   private final YoFrameVector2d adjustmentSolution = new YoFrameVector2d("adjustmentSolution", worldFrame, registry);

   private final FrameVector transferGradient = new FrameVector();
   private final FrameVector swingGradient = new FrameVector();

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);
   private final SimpleActiveSetQPSolverInterface activeSetSolver = new SimpleEfficientActiveSetQPSolver();

   private final FramePoint tempPoint = new FramePoint();
   private final FramePoint2d tempFinalCoM = new FramePoint2d();
   private final FramePoint2d tempPoint2d = new FramePoint2d();

   private final FrameVector tempVector = new FrameVector();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      maximumDesiredKneeBend.set(0.3);
      maximumNumberOfIterations.set(3);

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleLocations.put(robotSide, new FramePoint());

         YoFramePoint hipMaximumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMaximumPoint", worldFrame, registry);
         YoFramePoint hipMinimumLocation = new YoFramePoint(robotSide.getShortLowerCaseName() + "PredictedHipMinimumPoint", worldFrame, registry);
         hipMaximumLocations.put(robotSide, hipMaximumLocation);
         hipMinimumLocations.put(robotSide, hipMinimumLocation);
      }

      setupVisualizers(yoGraphicsListRegistry);

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
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame ankleFrame = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH).getFrameAfterJoint();
         Vector2dZUpFrame stepDirectionFrame = new Vector2dZUpFrame(robotSide.getShortLowerCaseName() + "Step Direction Frame", ankleFrame);
         stepDirectionFrames.put(robotSide, stepDirectionFrame);
      }

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

         yoGraphicsList.add(hipMaximumLocationViz);
         yoGraphicsList.add(hipMinimumLocationViz);
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

      boolean isStepReachable = checkReachabilityOfStep();

      while (!isStepReachable)
      {
         double requiredAdjustment = computeRequiredAdjustment();
         this.requiredAdjustment.set(requiredAdjustment);

         computeTransferGradient();
         computeSwingGradient();

         computeTimingAdjustment(requiredAdjustment);

         double initialTime = icpPlanner.getInitialTime();
         icpPlanner.adjustTransferDurations(transferTimeAdjustment.getDoubleValue());
         icpPlanner.adjustSwingDurations(swingTimeAdjustment.getDoubleValue());
         icpPlanner.initializeForTransfer(initialTime);

         isStepReachable = checkReachabilityOfStep();
         numberOfIterations.increment();

         if (numberOfIterations.getIntegerValue() > maximumNumberOfIterations.getIntegerValue())
            break;
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


   private final double perpendicularWeight = 0.1;
   private final double swingAdjustmentWeight = 5.0;
   private final double transferAdjustmentWeight = 5.0;

   private final DenseMatrix64F solution = new DenseMatrix64F(3,1);

   private final DenseMatrix64F solverInput_H = new DenseMatrix64F(2,2);
   private final DenseMatrix64F solverInput_h = new DenseMatrix64F(2,1);
   private final DenseMatrix64F solverInput_Aeq = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F solverInput_beq = new DenseMatrix64F(1, 1);

   private void computeTimingAdjustment(double requiredAdjustment)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();
      transferGradient.changeFrame(stepDirectionFrames.get(stanceSide));
      swingGradient.changeFrame(stepDirectionFrames.get(stanceSide));

      solution.zero();

      solverInput_H.zero();

      double transferPerpendicularGradient = transferGradient.getY();
      double transferParallelGradient = transferGradient.getX();
      double swingPerpendicularGradient = swingGradient.getY();
      double swingParallelGradient = swingGradient.getX();

      solverInput_H.set(0, 0, perpendicularWeight * Math.pow(transferPerpendicularGradient, 2.0) + transferAdjustmentWeight);
      solverInput_H.set(0, 1, perpendicularWeight * transferPerpendicularGradient * swingPerpendicularGradient);
      solverInput_H.set(1, 0, perpendicularWeight * transferPerpendicularGradient * swingPerpendicularGradient);
      solverInput_H.set(1, 1, perpendicularWeight * Math.pow(swingPerpendicularGradient, 2.0) + swingAdjustmentWeight);

      solverInput_Aeq.set(0, 0, transferParallelGradient);
      solverInput_Aeq.set(0, 1, swingParallelGradient);
      solverInput_beq.set(0, 0, requiredAdjustment);

      activeSetSolver.clear();

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, 0.0);
      activeSetSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      int numberOfIterations = activeSetSolver.solve(solution);

      if (MatrixTools.containsNaN(solution))
      {
         NoConvergenceException e = new NoConvergenceException(numberOfIterations);
         e.printStackTrace();
         PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
      }

      transferTimeAdjustment.set(solution.get(0, 0));
      swingTimeAdjustment.set(solution.get(1, 0));

      adjustmentSolution.setX(transferParallelGradient * transferTimeAdjustment.getDoubleValue() + swingParallelGradient * swingTimeAdjustment.getDoubleValue());
      adjustmentSolution.setY(transferPerpendicularGradient * transferTimeAdjustment.getDoubleValue() +
            swingPerpendicularGradient * swingTimeAdjustment.getDoubleValue());
   }




   private void updateFrames(Footstep nextFootstep)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();

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
      FramePoint upcomingStepLocation = ankleLocations.get(stanceSide.getOppositeSide());
      stanceAnkleLocation.setToZero(fullRobotModel.getLegJoint(stanceSide, LegJointName.ANKLE_PITCH).getFrameAfterJoint());
      nextFootstep.getPositionIncludingFrame(upcomingStepLocation);

      // get the step direction
      upcomingStepLocation.changeFrame(worldFrame);
      stanceAnkleLocation.changeFrame(worldFrame);
      tempVector.changeFrame(worldFrame);
      tempVector.set(upcomingStepLocation);
      tempVector.sub(stanceAnkleLocation);
      stepDirectionFrames.get(stanceSide).setXAxis(tempVector);

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
      double variation = TWIDDLE_SIZE;

      icpPlanner.adjustTransferDurations(variation);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      icpPlanner.adjustTransferDurations(-variation);
      icpPlanner.initializeForTransfer(currentInitialTime);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      yoTransferGradient.setByProjectionOntoXYPlane(transferGradient);
   }

   private void computeSwingGradient()
   {
      double currentInitialTime = icpPlanner.getInitialTime();
      double variation = TWIDDLE_SIZE;

      icpPlanner.adjustSwingDurations(variation);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      icpPlanner.adjustSwingDurations(-variation);
      icpPlanner.initializeForTransfer(currentInitialTime);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      swingGradient.setToZero(worldFrame);
      swingGradient.setXY(adjustedCoMPosition);
      swingGradient.sub(tempPoint);
      swingGradient.scale(1.0 / variation);

      yoSwingGradient.setByProjectionOntoXYPlane(swingGradient);
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
