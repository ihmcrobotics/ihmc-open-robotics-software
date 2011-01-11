package us.ihmc.commonWalkingControlModules.desiredStepLocation;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.SteppingStonesCaptureRegionIntersectionCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;



/**
 * <p>Title: SteppingStonesDesiredStepLocationCalculator</p>
 *
 * <p>Description: A DesiredStepLocationCalculator that tries to step in a direction, a given distance (with respect to the support foot)
 * but will adjust to be inside the stepping stones, and if this spot will lead to the robot falling, this will try to calculate a position
 * close to the desired point that will result
 * in the robot maintaining balance.</p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author Yobotics-IHMC biped team
 * @version 1.0
 */
public class SteppingStonesDesiredStepLocationCalculator implements DesiredStepLocationCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredStepLocationCalculator");

   private final SideDependentList<ReferenceFrame> footZUpFrames;
   private final DoubleYoVariable stanceWidth = new DoubleYoVariable("stanceWidth", registry);
   private final DoubleYoVariable stanceLength = new DoubleYoVariable("stanceLength", registry);
   private final DoubleYoVariable stepAngle = new DoubleYoVariable("stepAngle", registry);
   private final DoubleYoVariable stepDistance = new DoubleYoVariable("stepDistance", registry);
   private final DoubleYoVariable stepUp = new DoubleYoVariable("stepUp", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);

   private final DoubleYoVariable percentNominalToAdjusted = new DoubleYoVariable("percentNominalToAdjusted", registry);

   private final YoFramePoint adjustedStepPosition = new YoFramePoint("adjustedStepPosition", "", ReferenceFrame.getWorldFrame(), registry);

   private final SteppingStonesCaptureRegionIntersectionCalculator steppingStonesCaptureRegionIntersectionCalculator;

   private StepLocationScorer stepLocationScorer;

   private final CaptureRegionStepLocationSelectionMethod captureRegionStepLocationSelectionMethod = CaptureRegionStepLocationSelectionMethod.NEAREST_POINT;

   private boolean VISUALIZE = true;

   public SteppingStonesDesiredStepLocationCalculator(SteppingStones steppingStones, CommonWalkingReferenceFrames commonWalkingReferenceFrames,
           CaptureRegionCalculator captureRegionCalculator, YoVariableRegistry yoVariableRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, boolean registerVarList)
   {
      percentNominalToAdjusted.set(1.0);

      double defaultStepWidth = 0.22;
      stanceWidth.set(defaultStepWidth);
      double defaultStanceLength = CaptureRegionCalculator.kinematicRangeFromCoP * 0.5;    // 0.7;//0.40;
      stanceLength.set(defaultStanceLength);
      stepAngle.set(Math.atan(stanceWidth.getDoubleValue() / stanceLength.getDoubleValue()));    // 0.4
      stepDistance.set(Math.sqrt((stanceWidth.getDoubleValue() * stanceWidth.getDoubleValue())
                                 + (stanceLength.getDoubleValue() * stanceLength.getDoubleValue())));

      ReferenceFrame leftAnkleZUpFrame = commonWalkingReferenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.LEFT);
      ReferenceFrame rightAnkleZUpFrame = commonWalkingReferenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.RIGHT);

      footZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);

      stepLocationScorer = new WeightedDistanceScorer(captureRegionCalculator, footZUpFrames, stanceWidth, stanceLength, stepAngle, stepDistance);

//    stepLocationScorer = new BasicReachableScorer(yoboticsBipedCaptureRegionCalculator);

      if (steppingStones != null)
      {
         steppingStonesCaptureRegionIntersectionCalculator = new SteppingStonesCaptureRegionIntersectionCalculator(steppingStones, yoVariableRegistry,
                 dynamicGraphicObjectsListRegistry);
      }
      else
      {
         steppingStonesCaptureRegionIntersectionCalculator = null;
      }

      if ((yoVariableRegistry != null) && registerVarList)    // (VarListsToRegister.REGISTER_DESIRED_STEP_LOCATION_CALCULATOR))
      {
         yoVariableRegistry.addChild(registry);
      }

      if (dynamicGraphicObjectsListRegistry == null)
         VISUALIZE = false;

      if (VISUALIZE)
      {
         double scale = 0.012;
         DynamicGraphicPosition adjustedStepPositionDynamicGraphicPosition = new DynamicGraphicPosition("Adjusted Step Position", adjustedStepPosition, scale,
                                                                                YoAppearance.Yellow());

         dynamicGraphicObjectsListRegistry.registerArtifact("DesiredStepLocation", adjustedStepPositionDynamicGraphicPosition.createArtifact());

//       int level = 4;
//       YoboticsBipedPlotter.registerDynamicGraphicPosition("Adjusted Step Position", adjustedStepPositionDynamicGraphicPosition, level);
      }
   }

   public Footstep computeDesiredStepLocation(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FrameConvexPolygon2d captureRegion,
           FramePoint capturePoint)
   {
      FramePoint nominalLocation = getNominalStepLocation(supportLeg);
      Point2d nominalLocation2d = new Point2d(nominalLocation.getX(), nominalLocation.getY());

      if (captureRegion == null)
      {
         return new Footstep(supportLeg.getOppositeSide(), nominalLocation, stepYaw.getDoubleValue());
      }

      captureRegion = captureRegion.changeFrameCopy(ReferenceFrame.getWorldFrame());

      ArrayList<ConvexPolygon2d> captureRegionSteppingStonesIntersections = null;
      if (steppingStonesCaptureRegionIntersectionCalculator != null)
      {
         captureRegionSteppingStonesIntersections =
            steppingStonesCaptureRegionIntersectionCalculator.findIntersectionsBetweenSteppingStonesAndCaptureRegion(captureRegion);
      }

      switch (captureRegionStepLocationSelectionMethod)
      {
         case NEAREST_CENTROID :
            computeAndSetBestCentroidToStepTo(nominalLocation2d, captureRegionSteppingStonesIntersections);

            break;

         case NEAREST_POINT :
            computeAndSetBestNearestPointToStepTo(nominalLocation2d, captureRegionSteppingStonesIntersections);

            break;

         default :
            throw new RuntimeException("Enum constant not handled");
      }

      FrameVector nominalToAdjusted = new FrameVector(adjustedStepPosition.getFramePointCopy());
      nominalToAdjusted.sub(nominalLocation);
      nominalToAdjusted.scale(percentNominalToAdjusted.getDoubleValue());

      FramePoint locationToReturn = new FramePoint(nominalLocation);
      locationToReturn.add(nominalToAdjusted);

      return new Footstep(supportLeg.getOppositeSide(), locationToReturn, stepYaw.getDoubleValue());
   }

   public FramePoint getNominalStepLocation(RobotSide supportLeg)
   {
      double sideDependentStepAngle = supportLeg.negateIfLeftSide(stepAngle.getDoubleValue());

      ReferenceFrame supportLegAnkleZUpFrame = footZUpFrames.get(supportLeg);

      FramePoint desiredSwingToPosition = new FramePoint(supportLegAnkleZUpFrame);
      desiredSwingToPosition.setX(desiredSwingToPosition.getX() + stepDistance.getDoubleValue() * Math.cos(sideDependentStepAngle));
      desiredSwingToPosition.setY(desiredSwingToPosition.getY() + stepDistance.getDoubleValue() * Math.sin(sideDependentStepAngle));

      desiredSwingToPosition.setZ(desiredSwingToPosition.getZ() + stepUp.getDoubleValue());
      desiredSwingToPosition = desiredSwingToPosition.changeFrameCopy(ReferenceFrame.getWorldFrame());

      return desiredSwingToPosition;
   }

   public double getStepLocationScore(RobotSide supportLeg, Footstep desiredFootstep)
   {
      return stepLocationScorer.getStepLocationScore(supportLeg, desiredFootstep);
   }

   private void computeAndSetBestNearestPointToStepTo(Point2d nominalLocation2d, ArrayList<ConvexPolygon2d> captureRegionSteppingStonesIntersections)
   {
      Point2d nearestPoint = new Point2d();
      double nearestDistanceSquared = Double.POSITIVE_INFINITY;
      Point2d pointToTest = new Point2d();

      if (captureRegionSteppingStonesIntersections != null)    // If there are no captureRegionSteppingStonesIntersections, just keep stepping where you were before for now...
      {
         for (ConvexPolygon2d possiblePlaceToStep : captureRegionSteppingStonesIntersections)
         {
            pointToTest.set(nominalLocation2d);
            possiblePlaceToStep.orthogonalProjection(pointToTest);

            double possibleDistanceSquared = nominalLocation2d.distanceSquared(pointToTest);

            if (possibleDistanceSquared < nearestDistanceSquared)
            {
               nearestPoint.set(pointToTest);
               nearestDistanceSquared = possibleDistanceSquared;
            }
         }

         if (nearestDistanceSquared != Double.POSITIVE_INFINITY)    // If there are no near centroids, just keep stepping where you were before...
         {
            adjustedStepPosition.set(nearestPoint.x, nearestPoint.y, 0.0);
         }
      }

   }

   private void computeAndSetBestCentroidToStepTo(Point2d nominalLocation2d, ArrayList<ConvexPolygon2d> captureRegionSteppingStonesIntersections)
   {
      // For now, just step to the midpoint of the nearest one:
      Point2d nearestCentroid = null;
      double nearestDistanceSquared = Double.POSITIVE_INFINITY;
      Point2d centroid = new Point2d();

      if (captureRegionSteppingStonesIntersections != null)    // If there are no captureRegionSteppingStonesIntersections, just keep stepping where you were before for now...
      {
         for (ConvexPolygon2d possiblePlaceToStep : captureRegionSteppingStonesIntersections)
         {
            possiblePlaceToStep.getCentroid(centroid);
            double possibleDistanceSquared = nominalLocation2d.distanceSquared(centroid);

            if (possibleDistanceSquared < nearestDistanceSquared)
            {
               nearestCentroid = new Point2d(centroid);
               nearestDistanceSquared = possibleDistanceSquared;
            }
         }

         if (nearestCentroid != null)    // If there are no near centroids, just keep stepping where you were before...
         {
            adjustedStepPosition.set(nearestCentroid.x, nearestCentroid.y, 0.0);
         }
      }
   }

   private enum CaptureRegionStepLocationSelectionMethod {NEAREST_CENTROID, NEAREST_POINT}


   public void initializeAtStartOfSwing(RobotSide swingLegSide, CouplingRegistry couplingResitry)
   {
      // TODO Auto-generated method stub

   }
}
