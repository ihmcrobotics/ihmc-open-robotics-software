package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;

public class MultiStepCaptureRegionCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger stepsInQueue = new YoInteger("stepsInQueue", registry);
   private final YoInteger stepsConsideringForRecovery = new YoInteger("stepsConsideringForRecovery", registry);
   private final IntegerParameter maxStepsToConsider = new IntegerParameter("maxStepsToConsiderForRecovery", registry, 10);

   private final FrameConvexPolygon2D multiStepRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;

   private MultiStepCaptureRegionVisualizer visualizer = null;

   private static final double theoreticalMaxLength = 1.0;

   public MultiStepCaptureRegionCalculator(StepAdjustmentReachabilityConstraint reachabilityConstraint, YoRegistry parentRegistry)
   {
      this(reachabilityConstraint, parentRegistry, null);
   }

   public MultiStepCaptureRegionCalculator(StepAdjustmentReachabilityConstraint reachabilityConstraint, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.reachabilityConstraint = reachabilityConstraint;

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Multi Step Capture Region",
                                                                       yoMultiStepRegion,
                                                                       Color.YELLOW,
                                                                       false,
                                                                       false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);
      }

      parentRegistry.addChild(registry);
   }

   final FrameVector2D displacementOfVertex = new FrameVector2D();
   final FramePoint2D firstPointToAdd = new FramePoint2D();
   final FramePoint2D secondPointToAdd = new FramePoint2D();
   private final FrameLineSegment2D edgeToExtrude = new FrameLineSegment2D();
   private final FrameVector2D perp = new FrameVector2D();

   public void attachVisualizer(MultiStepCaptureRegionVisualizer visualizer)
   {
      this.visualizer = visualizer;
   }

   public void compute(RobotSide currentStanceSide, FrameConvexPolygon2DReadOnly oneStepCaptureRegion, double stepDuration, double omega, int stepsInQueue)
   {
      multiStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
      displacementOfVertex.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      stepsConsideringForRecovery.set(Math.min(stepsInQueue, maxStepsToConsider.getValue()));
      this.stepsInQueue.set(stepsInQueue);

      // The -1 at the end is because you've already taken the first step
      int stepsToTakeWithCurrentSupportSide = (int) Math.floor((stepsConsideringForRecovery.getIntegerValue() + 1) / 2) - 1;
      int stepsToTakeWithOppositeSupportSide = (int) Math.floor(stepsConsideringForRecovery.getIntegerValue() / 2);

      double stepExponential = Math.exp(-omega * stepDuration);
      double stepExponentialSquared = stepExponential * stepExponential;

      double currentSupportMultiplier = 0.0;
      double oppositeSupportMultiplier = 0.0;
      for (int step = 1; step <= stepsToTakeWithCurrentSupportSide; step++)
         currentSupportMultiplier = stepExponentialSquared * currentSupportMultiplier + stepExponentialSquared;
      for (int step = 1; step <= stepsToTakeWithOppositeSupportSide; step++)
         oppositeSupportMultiplier = stepExponentialSquared * oppositeSupportMultiplier + stepExponential;

      if (!oneStepCaptureRegion.isClockwiseOrdered())
         throw new RuntimeException("Does't work for counter clockwise yet");
      for (int i = 0; i < oneStepCaptureRegion.getNumberOfVertices(); i++)
      {
         edgeToExtrude.setIncludingFrame(oneStepCaptureRegion.getVertex(i), oneStepCaptureRegion.getNextVertex(i));
         edgeToExtrude.perpendicular(true, perp);
         computeNStepExpansionAlongStep(currentStanceSide,
                                        edgeToExtrude,
                                        displacementOfVertex,
                                        currentSupportMultiplier,
                                        oppositeSupportMultiplier);

         firstPointToAdd.setIncludingFrame(oneStepCaptureRegion.getVertex(i));
         firstPointToAdd.add(displacementOfVertex);

         secondPointToAdd.setIncludingFrame(oneStepCaptureRegion.getNextVertex(i));
         secondPointToAdd.add(displacementOfVertex);

         if (visualizer != null)
            visualizer.visualizeProcess(firstPointToAdd, secondPointToAdd, edgeToExtrude, oneStepCaptureRegion, i);

         multiStepRegion.addVertex(firstPointToAdd);
         multiStepRegion.addVertex(secondPointToAdd);
      }

      multiStepRegion.update();
      yoMultiStepRegion.setMatchingFrame(multiStepRegion, false);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoMultiStepRegion;
   }

   private final FramePoint2D origin = new FramePoint2D();
   private final FrameVector2D bestStepForStance = new FrameVector2D();
   private final FrameVector2D bestStepForSwing = new FrameVector2D();

   private void computeNStepExpansionAlongStep(RobotSide currentStanceSide,
                                               FrameLineSegment2DReadOnly edgeToExtrude,
                                               FrameVector2DBasics expansionToPack,
                                               double currentSupportMultiplier,
                                               double oppositeSupportMultiplier)
   {

      origin.setToZero(edgeToExtrude.getReferenceFrame());
      bestStepForStance.setReferenceFrame(edgeToExtrude.getReferenceFrame());
      bestStepForSwing.setReferenceFrame(edgeToExtrude.getReferenceFrame());

      getBestStepForSide(currentStanceSide, edgeToExtrude, bestStepForStance);
      getBestStepForSide(currentStanceSide.getOppositeSide(), edgeToExtrude, bestStepForSwing);

      expansionToPack.setAndScale(currentSupportMultiplier, bestStepForStance);
      expansionToPack.scaleAdd(oppositeSupportMultiplier, bestStepForSwing, expansionToPack);
   }

   private final ConvexPolygon2D reachabilityPolygon = new ConvexPolygon2D();
   private final Point2D stancePosition = new Point2D();

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private void getBestStepForSide(RobotSide supportSide, FrameLineSegment2DReadOnly edgeToExtrude, FrameVector2DBasics stepToPack)
   {
      stancePosition.setToZero();
      reachabilityPolygon.set(reachabilityConstraint.getReachabilityPolygonInFootFrame(supportSide));
      reachabilityPolygon.addVertex(stancePosition);
      reachabilityPolygon.update();

      edgeToExtrude.perpendicular(true, perp);
      perp.scale(3.0);
      transform.getTranslation().set(edgeToExtrude.midpoint());
      transform.getTranslation().add(perp.getX(), perp.getY(), 0.0);

      reachabilityPolygon.applyTransform(transform);
      stancePosition.applyTransform(transform);

      int closestIndex = -1;
      int altClosestIndex = -1;
      double closestDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < reachabilityPolygon.getNumberOfVertices(); i++)
      {
         double distance = edgeToExtrude.distanceSquared(reachabilityPolygon.getVertex(i));
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestIndex = i;
            altClosestIndex = -1;
         }
         else if (MathTools.epsilonEquals(distance, closestDistance, 1e-4))
         {
            altClosestIndex = i;
         }
      }

      if (altClosestIndex == -1)
      {
         stepToPack.sub(stancePosition, reachabilityPolygon.getVertex(closestIndex));
      }
      else
      {
         EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(stancePosition, reachabilityPolygon.getVertex(closestIndex),
                                                                 reachabilityPolygon.getVertex(altClosestIndex),
                                                                 origin);
         stepToPack.sub(stancePosition, origin);
      }
   }
}
