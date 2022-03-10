package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultiStepCaptureRegionCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger stepsInQueue = new YoInteger("stepsInQueue", registry);
   private final YoInteger stepsConsideringForRecovery = new YoInteger("stepsConsideringForRecovery", registry);
   private final IntegerParameter maxStepsToConsider = new IntegerParameter("maxStepsToConsiderForRecovery", registry, 3);

   private final FrameConvexPolygon2D multiStepRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;

   public MultiStepCaptureRegionCalculator(StepAdjustmentReachabilityConstraint reachabilityConstraint, YoRegistry parentRegistry)
   {
      this.reachabilityConstraint = reachabilityConstraint;

      parentRegistry.addChild(registry);
   }

   private final FrameVector2D globalPreviousNormal = new FrameVector2D();
   private final FrameVector2D globalThisNormal = new FrameVector2D();
   private final FrameVector2D displacementOfVertex = new FrameVector2D();
   private final FrameLine2D line = new FrameLine2D();

   public void setStepsInQueue(int stepsInQueue)
   {
      this.stepsInQueue.set(stepsInQueue);
   }

   public void compute(RobotSide currentStanceSide, FrameConvexPolygon2DReadOnly oneStepCaptureRegion, double stepDuration, double omega, int stepsInQueue)
   {
      multiStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
      stepsConsideringForRecovery.set(Math.min(stepsInQueue, maxStepsToConsider.getValue()));

      FrameVector2D distanceToExtrudePreviousNode = null;
      for (int i = 0; i < oneStepCaptureRegion.getNumberOfVertices(); i++)
      {
         if (distanceToExtrudePreviousNode == null)
         {
            line.setIncludingFrame(oneStepCaptureRegion.getPreviousVertex(i), oneStepCaptureRegion.getVertex(i));
            globalPreviousNormal.setIncludingFrame(line.perpendicularVector());
            computeNStepExpansionAlongStep(currentStanceSide, globalPreviousNormal, globalPreviousNormal, stepDuration, omega);
            distanceToExtrudePreviousNode = globalPreviousNormal;
         }

         line.setIncludingFrame(oneStepCaptureRegion.getVertex(i), oneStepCaptureRegion.getNextVertex(i));
         globalThisNormal.setIncludingFrame(line.perpendicularVector());

         computeNStepExpansionAlongStep(currentStanceSide, globalThisNormal, globalThisNormal, stepDuration, omega);

         combineTwoExtrusionsToGetOneDisplacement(distanceToExtrudePreviousNode, globalThisNormal, displacementOfVertex);

         multiStepRegion.addVertex(oneStepCaptureRegion.getVertex(i));
         multiStepRegion.getVertexUnsafe(i).add(displacementOfVertex);

         distanceToExtrudePreviousNode = globalThisNormal;
      }

      multiStepRegion.update();
      yoMultiStepRegion.setMatchingFrame(multiStepRegion, false);
   }

   private static void combineTwoExtrusionsToGetOneDisplacement(FrameVector2DReadOnly extrusionA,
                                                                FrameVector2DReadOnly extrusionB,
                                                                FrameVector2DBasics combinedExtrusion)
   {
      double crossProduct = extrusionB.cross(extrusionA);
      double lengthA = extrusionA.length();
      double lengthB = extrusionB.length();

      combinedExtrusion.setReferenceFrame(extrusionA.getReferenceFrame());
      combinedExtrusion.setX(extrusionA.getY() * lengthB - extrusionB.getY() * lengthA);
      combinedExtrusion.setY(extrusionB.getX() * lengthA - extrusionA.getX() * lengthB);
      combinedExtrusion.scale(1.0 / crossProduct);
   }

   private final FrameConvexPolygon2D reachabilityPolygon = new FrameConvexPolygon2D();
   private final FrameLine2D stepDirection = new FrameLine2D();
   private final FramePoint2D intersectionOne = new FramePoint2D();
   private final FramePoint2D intersectionTwo = new FramePoint2D();
   private final FramePoint2D origin = new FramePoint2D();

   private void computeNStepExpansionAlongStep(RobotSide currentStanceSide,
                                               FrameVector2DReadOnly directionToExpand,
                                               FrameVector2DBasics expansionToPack,
                                               double stepDuration,
                                               double omega)
   {
      double stepExponential = Math.exp(-omega * stepDuration);

      double maxLengthAlongDistance = 0.0;
      stepDirection.setToZero(directionToExpand.getReferenceFrame());
      stepDirection.getDirection().set(directionToExpand);
      origin.setToZero(directionToExpand.getReferenceFrame());

      for (int step = 1; step < stepsConsideringForRecovery.getIntegerValue(); step++)
      {
         reachabilityPolygon.setIncludingFrame(reachabilityConstraint.getReachabilityPolygonInFootFrame(currentStanceSide));
         reachabilityPolygon.changeFrameAndProjectToXYPlane(directionToExpand.getReferenceFrame());

         double lengthAlongDistance;
         int numberOfIntersections = reachabilityPolygon.intersectionWithRay(stepDirection, intersectionOne, intersectionTwo);

         if (numberOfIntersections == 0)
         {
            lengthAlongDistance = -reachabilityPolygon.distance(origin);
         }
         else if (numberOfIntersections == 1)
         {
            lengthAlongDistance = intersectionOne.distanceFromOrigin();
         }
         else
         {
            lengthAlongDistance = intersectionTwo.distanceFromOrigin();
         }

         maxLengthAlongDistance += (maxLengthAlongDistance + lengthAlongDistance) * stepExponential;
      }

      expansionToPack.set(directionToExpand);
      expansionToPack.scale(maxLengthAlongDistance / expansionToPack.length());
   }
}
