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
   private static final double TRANSFER_TWIDDLE_SIZE = 0.6;
   private static final double SWING_TWIDDLE_SIZE = 0.6;

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

   private final DoubleYoVariable minimumInitialTransferDuration = new DoubleYoVariable("minimumInitialTransferDuration", registry);
   private final DoubleYoVariable minimumEndTransferDuration = new DoubleYoVariable("minimumEndTransferDuration", registry);
   private final DoubleYoVariable minimumInitialSwingDuration = new DoubleYoVariable("minimumInitialSwingDuration", registry);
   private final DoubleYoVariable minimumEndSwingDuration = new DoubleYoVariable("minimumEndSwingDuration", registry);
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

   private static final double perpendicularWeight = 0.01;
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

   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solution;

   public DynamicReachabilityCalculator(ICPPlanner icpPlanner, FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlanner = icpPlanner;
      this.fullRobotModel = fullRobotModel;

      maximumDesiredKneeBend.set(0.1);
      maximumNumberOfIterations.set(3);

      minimumInitialTransferDuration.set(0.05);
      minimumEndTransferDuration.set(0.05);
      minimumInitialSwingDuration.set(0.15);
      minimumEndSwingDuration.set(0.15);
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
      for (int i = 0; i < numberOfFootstepsToConsider - 3; i++)
      {
         YoFrameVector2d higherTransferGradient = new YoFrameVector2d("higherTransferGradient" + i, worldFrame, registry);
         YoFrameVector2d higherSwingGradient = new YoFrameVector2d("higherSwingGradient" + i, worldFrame, registry);
         higherTransferGradients.add(higherTransferGradient);
         higherSwingGradients.add(higherSwingGradient);
      }


      solverInput_H = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
      solverInput_h = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

      solverInput_Lb = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);
      solverInput_Ub = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

      solverInput_Ain = new DenseMatrix64F(6, 2 * numberOfFootstepsToConsider);
      solverInput_bin = new DenseMatrix64F(6, 1);

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
      numberOfIterations.set(0);

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
   
   //// TODO: 3/22/17  this needs to work in single support too 

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
         double requiredAdjustment;
         if (needToMoveCoMBackward)
            requiredAdjustment = computeRequiredAdjustmentBackward();
         else
            requiredAdjustment = computeRequiredAdjustmentForward();
         this.requiredAdjustment.set(requiredAdjustment);

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




   private void computeTimingAdjustment(double requiredAdjustment)
   {
      RobotSide stanceSide = nextFootstep.getRobotSide().getOppositeSide();

      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      reshape();

      extractGradient(currentInitialTransferGradient, stanceSide);

      double currentInitialTransferPerpendicularGradient = transferGradient.getY();
      double currentInitialTransferParallelGradient = transferGradient.getX();

      extractGradient(currentEndTransferGradient, stanceSide);

      double currentEndTransferPerpendicularGradient = transferGradient.getY();
      double currentEndTransferParallelGradient = transferGradient.getX();

      extractGradient(initialSwingGradient, stanceSide);

      double swingInitialPerpendicularGradient = transferGradient.getY();
      double swingInitialParallelGradient = transferGradient.getX();

      extractGradient(endSwingGradient, stanceSide);

      double swingEndPerpendicularGradient = transferGradient.getY();
      double swingEndParallelGradient = transferGradient.getX();

      extractGradient(nextInitialTransferGradient, stanceSide);

      double nextInitialTransferPerpendicularGradient = transferGradient.getY();
      double nextInitialTransferParallelGradient = transferGradient.getX();

      extractGradient(nextEndTransferGradient, stanceSide);

      double nextEndTransferPerpendicularGradient = transferGradient.getY();
      double nextEndTransferParallelGradient = transferGradient.getX();

      parallel_J.set(0, 0, currentInitialTransferParallelGradient);
      parallel_J.set(0, 1, currentEndTransferParallelGradient);
      parallel_J.set(0, 2, swingInitialParallelGradient);
      parallel_J.set(0, 3, swingEndParallelGradient);
      parallel_J.set(0, 4, nextInitialTransferParallelGradient);
      parallel_J.set(0, 5, nextEndTransferParallelGradient);

      perpendicular_J.set(0, 0, currentInitialTransferPerpendicularGradient);
      perpendicular_J.set(0, 1, currentEndTransferPerpendicularGradient);
      perpendicular_J.set(0, 2, swingInitialPerpendicularGradient);
      perpendicular_J.set(0, 3, swingEndPerpendicularGradient);
      perpendicular_J.set(0, 4, nextInitialTransferPerpendicularGradient);
      perpendicular_J.set(0, 5, nextEndTransferPerpendicularGradient);

      adjustmentObjective_H.set(0, 0, 2 * transferAdjustmentWeight);
      adjustmentObjective_H.set(0, 1, transferAdjustmentWeight);
      adjustmentObjective_H.set(1, 0, transferAdjustmentWeight);
      adjustmentObjective_H.set(1, 1, 2 * transferAdjustmentWeight);
      adjustmentObjective_H.set(2, 2, 2 * swingAdjustmentWeight);
      adjustmentObjective_H.set(2, 3, swingAdjustmentWeight);
      adjustmentObjective_H.set(3, 2, swingAdjustmentWeight);
      adjustmentObjective_H.set(3, 3, 2 * swingAdjustmentWeight);
      adjustmentObjective_H.set(4, 4, 2 * transferAdjustmentWeight);
      adjustmentObjective_H.set(4, 5, transferAdjustmentWeight);
      adjustmentObjective_H.set(5, 4, transferAdjustmentWeight);
      adjustmentObjective_H.set(5, 5, 2 * transferAdjustmentWeight);

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


      // define bounds and timing constraints
      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      double currentTransferAlpha = icpPlanner.getTransferDurationAlpha(0);
      double currentTransferInitialDuration = currentTransferAlpha * currentTransferDuration;
      double currentTransferEndDuration = (1.0 - currentTransferAlpha) * currentTransferDuration;

      double currentSwingDuration = icpPlanner.getSwingDuration(0);
      double currentSwingAlpha = icpPlanner.getSwingDurationAlpha(0);
      double currentSwingInitialDuration = currentSwingAlpha * currentSwingDuration;
      double currentSwingEndDuration = (1.0 - currentSwingAlpha) * currentSwingDuration;

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      double nextTransferAlpha = icpPlanner.getTransferDurationAlpha(1);
      double nextTransferInitialDuration = nextTransferAlpha * nextTransferDuration;
      double nextTransferEndDuration = (1.0 - nextTransferAlpha) * nextTransferDuration;

      solverInput_Lb.set(0, 0, minimumInitialTransferDuration.getDoubleValue() - currentTransferInitialDuration);
      solverInput_Lb.set(1, 0, minimumEndTransferDuration.getDoubleValue() - currentTransferEndDuration);
      solverInput_Lb.set(2, 0, minimumInitialSwingDuration.getDoubleValue() - currentSwingInitialDuration);
      solverInput_Lb.set(3, 0, minimumEndSwingDuration.getDoubleValue() - currentSwingEndDuration);
      solverInput_Lb.set(4, 0, minimumInitialTransferDuration.getDoubleValue() - nextTransferInitialDuration);
      solverInput_Lb.set(5, 0, minimumEndTransferDuration.getDoubleValue() - nextTransferEndDuration);

      solverInput_Ub.set(0, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(1, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(2, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(3, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(4, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(5, 0, Double.POSITIVE_INFINITY);

      /*
      solverInput_Ain.set(0, 0, -1.0);
      solverInput_Ain.set(0, 1, -1.0);
      solverInput_Ain.set(1, 0, -1.0);
      solverInput_Ain.set(1, 1, -1.0);
      solverInput_Ain.set(2, 0, -1.0);
      solverInput_Ain.set(2, 1, -1.0);

      solverInput_bin.set(0, 0, -minimumTransferDuration.getDoubleValue() + (currentTransferInitialDuration + currentTransferEndDuration));
      solverInput_bin.set(1, 0, -minimumSwingDuration.getDoubleValue() + (currentSwingInitialDuration + currentSwingEndDuration));
      solverInput_bin.set(2, 0, -minimumTransferDuration.getDoubleValue() + (nextTransferInitialDuration + nextTransferEndDuration));

      solverInput_Ain.set(3, 0, 1.0);
      solverInput_Ain.set(3, 1, 1.0);
      solverInput_Ain.set(4, 0, 1.0);
      solverInput_Ain.set(4, 1, 1.0);
      solverInput_Ain.set(5, 0, 1.0);
      solverInput_Ain.set(5, 1, 1.0);

      solverInput_bin.set(3, 0, maximumTransferDuration.getDoubleValue() - (currentTransferInitialDuration + currentTransferEndDuration));
      solverInput_bin.set(4, 0, maximumSwingDuration.getDoubleValue() - (currentSwingInitialDuration + currentSwingEndDuration));
      solverInput_bin.set(5, 0, maximumTransferDuration.getDoubleValue() - (nextTransferInitialDuration + nextTransferEndDuration));
      */

      activeSetSolver.clear();

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, scalar);
      activeSetSolver.setVariableBounds(solverInput_Lb, solverInput_Ub);
      //activeSetSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      int numberOfIterations = activeSetSolver.solve(solution);

      if (MatrixTools.containsNaN(solution))
      {
         NoConvergenceException e = new NoConvergenceException(numberOfIterations);
         e.printStackTrace();
         PrintTools.warn(this, "Only showing the stack trace of the first " + e.getClass().getSimpleName() + ". This may be happening more than once.");
      }

      extractSolution(solution);
   }


   private void reshape()
   {
      int numberOfFootstepsToConsider = icpPlanner.getNumberOfFootstepsToConsider();
      int numberOfFootstepsRegistered = icpPlanner.getNumberOfFootstepsRegistered();

      int problemSize = 6;

      //// TODO: 3/24/17
      /*
      if (numberOfFootstepsToConsider > 3 & numberOfFootstepsRegistered > 2)
         problemSize += 2 * Math.min(numberOfFootstepsToConsider - 3, numberOfFootstepsRegistered - 1);
         */

      solution.reshape(problemSize, problemSize);

      adjustmentObjective_H.reshape(problemSize, problemSize);
      perpendicularObjective_H.reshape(problemSize, problemSize);
      parallelObjective_H.reshape(problemSize, problemSize);
      parallelObjective_h.reshape(problemSize, 1);

      parallel_J.reshape(1, problemSize);
      perpendicular_J.reshape(1, problemSize);

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      solverInput_Lb.reshape(problemSize, 1);
      solverInput_Ub.reshape(problemSize, 1);

      solverInput_Ain.reshape(6, problemSize);
      solverInput_bin.reshape(6, 1);

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

      solverInput_Ain.zero();
      solverInput_bin.zero();
   }

   private void extractGradient(YoFrameVector2d gradientToExtract, RobotSide stanceSide)
   {
      transferGradient.setToZero(worldFrame);
      gradientToExtract.getFrameTuple2d(tempVector2d);
      transferGradient.setXY(tempVector2d);
      transferGradient.changeFrame(stepDirectionFrames.get(stanceSide));
   }

   private void extractSolution(DenseMatrix64F solution)
   {
      // handle current transfer
      double currentInitialTransferAdjustment = solution.get(0, 0);
      double currentEndTransferAdjustment = solution.get(1, 0);

      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      double currentTransferAlpha = icpPlanner.getTransferDurationAlpha(0);
      double currentInitialTransferDuration = currentTransferAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferAlpha) * currentTransferDuration;

      currentInitialTransferDuration += currentInitialTransferAdjustment;
      currentEndTransferDuration += currentEndTransferAdjustment;
      currentTransferAdjustment.set(currentInitialTransferAdjustment + currentEndTransferAdjustment);
      this.currentTransferAlpha.set(currentInitialTransferDuration / (currentInitialTransferDuration + currentEndTransferDuration));

      // handle current swing
      double initialSwingAdjustment = solution.get(2, 0);
      double endSwingAdjustment = solution.get(3, 0);

      double swingDuration = icpPlanner.getSwingDuration(0);
      double swingAlpha = icpPlanner.getSwingDurationAlpha(0);
      double swingInitialDuration = swingAlpha * swingDuration;
      double swingEndDuration = (1.0 - swingAlpha) * swingDuration;

      swingInitialDuration += initialSwingAdjustment;
      swingEndDuration += endSwingAdjustment;
      swingAdjustment.set(initialSwingAdjustment + endSwingAdjustment);
      this.swingAlpha.set(swingInitialDuration / (swingInitialDuration + swingEndDuration));


      // handle next transfer
      double nextInitialTransferAdjustment = solution.get(4, 0);
      double nextEndTransferAdjustment = solution.get(5, 0);

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
      double currentInitialTime = icpPlanner.getInitialTime();

      double currentTransferDuration = icpPlanner.getTransferDuration(0);
      double currentTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(0);

      double currentInitialTransferDuration = currentTransferDurationAlpha * currentTransferDuration;
      double currentEndTransferDuration = (1.0 - currentTransferDurationAlpha) * currentTransferDuration;

      // compute initial transfer duration gradient
      double variation = TRANSFER_TWIDDLE_SIZE * currentInitialTransferDuration;
      double modifiedTransferDurationAlpha = (currentInitialTransferDuration + variation) / (currentTransferDuration + variation);

      icpPlanner.adjustTransferDuration(0, variation);
      icpPlanner.setTransferDurationAlpha(0, modifiedTransferDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      //// TODO: 3/24/17  review this
      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      currentInitialTransferGradient.setByProjectionOntoXYPlane(transferGradient);

      // compute end transfer duration gradient
      icpPlanner.adjustTransferDuration(0, -variation);
      variation = TRANSFER_TWIDDLE_SIZE * currentEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (currentEndTransferDuration + variation) / (currentTransferDuration + variation);

      icpPlanner.adjustTransferDuration(0, variation);
      icpPlanner.setTransferDurationAlpha(0, modifiedTransferDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      currentEndTransferGradient.setByProjectionOntoXYPlane(transferGradient);

      // reset everything to normal
      icpPlanner.adjustTransferDuration(0, -variation);
      icpPlanner.setTransferDurationAlpha(0, currentTransferDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);
   }

   private void computeNextTransferGradient()
   {
      boolean isThisTheFinalTransfer = icpPlanner.getNumberOfFootstepsRegistered() == 1;

      double currentInitialTime = icpPlanner.getInitialTime();

      double nextTransferDuration = icpPlanner.getTransferDuration(1);
      double nextTransferDurationAlpha = icpPlanner.getTransferDurationAlpha(1);

      double nextInitialTransferDuration = nextTransferDurationAlpha * nextTransferDuration;
      double nextEndTransferDuration = (1.0 - nextTransferDurationAlpha) * nextTransferDuration;

      // compute initial transfer duration gradient
      double variation = TRANSFER_TWIDDLE_SIZE * nextInitialTransferDuration;
      double modifiedTransferDurationAlpha = (nextInitialTransferDuration + variation) / (nextTransferDuration + variation);

      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(nextTransferDuration + variation);
         icpPlanner.setFinalTransferDurationAlpha(modifiedTransferDurationAlpha);
      }
      else
      {
         icpPlanner.adjustTransferDuration(1, variation);
         icpPlanner.setTransferDurationAlpha(1, modifiedTransferDurationAlpha);
      }
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      nextInitialTransferGradient.setByProjectionOntoXYPlane(transferGradient);

      // compute end transfer duration gradient
      if (isThisTheFinalTransfer)
         icpPlanner.setFinalTransferDuration(nextTransferDuration);
      else
         icpPlanner.adjustTransferDuration(1, -variation);

      variation = TRANSFER_TWIDDLE_SIZE * nextEndTransferDuration;
      modifiedTransferDurationAlpha = 1.0 - (nextEndTransferDuration + variation) / (nextTransferDuration + variation);

      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(nextTransferDuration + variation);
         icpPlanner.setFinalTransferDurationAlpha(modifiedTransferDurationAlpha);
      }
      else
      {
         icpPlanner.adjustTransferDuration(1, variation);
         icpPlanner.setTransferDurationAlpha(1, modifiedTransferDurationAlpha);
      }
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      nextEndTransferGradient.setByProjectionOntoXYPlane(transferGradient);

      // reset everything to normal
      if (isThisTheFinalTransfer)
      {
         icpPlanner.setFinalTransferDuration(nextTransferDuration);
         icpPlanner.setFinalTransferDurationAlpha(nextTransferDurationAlpha);
      }
      else
      {
         icpPlanner.adjustTransferDuration(1, -variation);
         icpPlanner.setTransferDurationAlpha(1, nextTransferDurationAlpha);
      }
      icpPlanner.initializeForTransfer(currentInitialTime);
   }

   private void computeSwingGradient()
   {
      double currentInitialTime = icpPlanner.getInitialTime();

      double currentSwingDuration = icpPlanner.getSwingDuration(0);
      double currentSwingDurationAlpha = icpPlanner.getSwingDurationAlpha(0);

      double currentInitialSwingDuration = currentSwingDurationAlpha * currentSwingDuration;
      double currentEndSwingDuration = (1.0 - currentSwingDurationAlpha) * currentSwingDuration;

      // compute initial transfer duration gradient
      double variation = SWING_TWIDDLE_SIZE * currentInitialSwingDuration;
      double modifiedSwingDurationAlpha = (currentInitialSwingDuration + variation) / (currentSwingDuration + variation);

      icpPlanner.adjustSwingDuration(0, variation);
      icpPlanner.setSwingDurationAlpha(0, modifiedSwingDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      initialSwingGradient.setByProjectionOntoXYPlane(transferGradient);

      // compute end transfer duration gradient
      icpPlanner.adjustTransferDuration(0, -variation);
      variation = SWING_TWIDDLE_SIZE * currentEndSwingDuration;
      modifiedSwingDurationAlpha = 1.0 - (currentEndSwingDuration + variation) / (currentSwingDuration + variation);

      icpPlanner.adjustSwingDuration(0, variation);
      icpPlanner.setSwingDurationAlpha(0, modifiedSwingDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);

      icpPlanner.getFinalDesiredCenterOfMassPosition(adjustedCoMPosition);

      predictedCoMPosition.changeFrame(worldFrame);
      tempPoint.setToZero(worldFrame);
      tempPoint.set(predictedCoMPosition);
      tempPoint.setZ(0.0);
      transferGradient.setToZero(worldFrame);
      transferGradient.setXY(adjustedCoMPosition);
      transferGradient.sub(tempPoint);
      transferGradient.scale(1.0 / variation);

      endSwingGradient.setByProjectionOntoXYPlane(transferGradient);

      // reset everything to normal
      icpPlanner.adjustSwingDuration(0, -variation);
      icpPlanner.setTransferDurationAlpha(0, currentSwingDurationAlpha);
      icpPlanner.initializeForTransfer(currentInitialTime);
   }
   
   //// TODO: 3/24/17  
   private void computeHigherTransferGradients()
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

         higherTransferGradients.get(stepIndex).setByProjectionOntoXYPlane(transferGradient);
      }
   }

   //// TODO: 3/24/17  
   private void computeHigherSwingGradients()
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

         higherSwingGradients.get(stepIndex).setByProjectionOntoXYPlane(swingGradient);
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
