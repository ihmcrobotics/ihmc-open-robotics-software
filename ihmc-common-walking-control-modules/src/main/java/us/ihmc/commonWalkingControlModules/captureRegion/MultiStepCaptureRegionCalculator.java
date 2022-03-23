package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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

/**
 * This class computes the N-Step capture region, but it does so using the reachability constraint. This is to address the fact that you cannot
 * arbitrarily step at the maximum length, like is typically done when calculating the N-Step region. For example, steps that are perfectly sideways are bad if
 * you can't cross over, because only one side will allow you to step towards the goal. In this way, this class computes the reachability aware N-Step
 * capture region.
 */
public class MultiStepCaptureRegionCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger stepsInQueue = new YoInteger("stepsInQueue", registry);
   private final YoInteger stepsConsideringForRecovery = new YoInteger("stepsConsideringForRecovery", registry);
   private final IntegerParameter maxStepsToConsider = new IntegerParameter("maxStepsToConsiderForRecovery", registry, 10);

   private final FrameConvexPolygon2D multiStepRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;

   final FrameVector2D vertexExtrusionVector = new FrameVector2D();
   final FramePoint2D extrudedFirstVertex = new FramePoint2D();
   final FramePoint2D extrudedSecondVertex = new FramePoint2D();
   private final FrameLineSegment2D edgeToExtrude = new FrameLineSegment2D();
   private final FrameVector2D vectorPerpendicularToEdge = new FrameVector2D();

   private MultiStepCaptureRegionVisualizer visualizer = null;

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

   public void attachVisualizer(MultiStepCaptureRegionVisualizer visualizer)
   {
      this.visualizer = visualizer;
   }

   public void reset()
   {
      yoMultiStepRegion.clear();
   }

   /**
    * Computes the reachability aware NStep capture region
    * @param currentStanceSide foot side that is currently on the ground
    * @param oneStepCaptureRegion one-step capture region to expand
    * @param stepDuration total step duration. This is the sum of the swing and transfer duration.
    * @param omega time constant of the LIP
    * @param stepsInQueue number of steps you are allowed to take
    */
   public void compute(RobotSide currentStanceSide, FrameConvexPolygon2DReadOnly oneStepCaptureRegion, double stepDuration, double omega, int stepsInQueue)
   {
      multiStepRegion.clear(oneStepCaptureRegion.getReferenceFrame());
      vertexExtrusionVector.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      stepsConsideringForRecovery.set(Math.min(stepsInQueue, maxStepsToConsider.getValue()));
      this.stepsInQueue.set(stepsInQueue);

      // The -1 at the end is because you've already taken the first step, so don't consider it.
      int stepsToTakeWithCurrentSupportSide = (int) Math.floor((stepsConsideringForRecovery.getIntegerValue() + 1) / 2) - 1;
      int stepsToTakeWithOppositeSupportSide = (int) Math.floor(stepsConsideringForRecovery.getIntegerValue() / 2);

      // Compute the time scaling effects of the future steps.
      double stepExponential = Math.exp(-omega * stepDuration);
      double stepExponentialSquared = stepExponential * stepExponential;

      // Compute the maximum effect that stepping with each side will have on increasing the capture region, normalized by length. This is just a
      // concatenation of taking all of those steps.
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
         // compute the current edge of the capture region, which will then be extruded in a certain direction.
         edgeToExtrude.setIncludingFrame(oneStepCaptureRegion.getVertex(i), oneStepCaptureRegion.getNextVertex(i));

         // compute how much additional capturability you get along that edge by considering how additional steps can be taken. These also consider the
         // reachability constraints of the robot during that expansion.
         computeNStepExpansionAlongStep(currentStanceSide,
                                        edgeToExtrude, vertexExtrusionVector,
                                        currentSupportMultiplier,
                                        oppositeSupportMultiplier);

         // extrude that edge
         extrudedFirstVertex.setIncludingFrame(oneStepCaptureRegion.getVertex(i));
         extrudedFirstVertex.add(vertexExtrusionVector);

         extrudedSecondVertex.setIncludingFrame(oneStepCaptureRegion.getNextVertex(i));
         extrudedSecondVertex.add(vertexExtrusionVector);

         if (visualizer != null)
            visualizer.visualizeProcess(extrudedFirstVertex, extrudedSecondVertex, edgeToExtrude, oneStepCaptureRegion, i);

         // add the vertices of the extruded edge
         multiStepRegion.addVertex(extrudedFirstVertex);
         multiStepRegion.addVertex(extrudedSecondVertex);
      }

      multiStepRegion.update();
      yoMultiStepRegion.setMatchingFrame(multiStepRegion, false);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoMultiStepRegion;
   }

   private final FramePoint2D origin = new FramePoint2D();
   private final FrameVector2D bestStepForStanceSide = new FrameVector2D();
   private final FrameVector2D bestStepForSwingSide = new FrameVector2D();

   private void computeNStepExpansionAlongStep(RobotSide currentStanceSide,
                                               FrameLineSegment2DReadOnly edgeToExtrude,
                                               FrameVector2DBasics expansionToPack,
                                               double currentSupportMultiplier,
                                               double oppositeSupportMultiplier)
   {

      origin.setToZero(edgeToExtrude.getReferenceFrame());
      bestStepForStanceSide.setReferenceFrame(edgeToExtrude.getReferenceFrame());
      bestStepForSwingSide.setReferenceFrame(edgeToExtrude.getReferenceFrame());

      // Compute the step for each side that would best help recover from additiional error in that direction.
      getBestStepForSide(currentStanceSide, edgeToExtrude, bestStepForStanceSide);
      getBestStepForSide(currentStanceSide.getOppositeSide(), edgeToExtrude, bestStepForSwingSide);

      // Scale those step lengths based on the exponential time effects of the step durations, which is just up-integrated the LIP dynamics.
      expansionToPack.setAndScale(currentSupportMultiplier, bestStepForStanceSide);
      expansionToPack.scaleAdd(oppositeSupportMultiplier, bestStepForSwingSide, expansionToPack);
   }

   private final ConvexPolygon2D reachabilityPolygon = new ConvexPolygon2D();
   private final Point2D stancePosition = new Point2D();

   private final RigidBodyTransform transform = new RigidBodyTransform();

   /**
    * Computes the step vector that maximizes the distance away from the capture region the robot can step and still be N-Step capturable. This is the
    * point in the reachability polygon that would first contact the capture region, if it is moved in that direction.
    */
   private void getBestStepForSide(RobotSide supportSide, FrameLineSegment2DReadOnly edgeToExtrude, FrameVector2DBasics stepToPack)
   {
      // compute the reachability polygon for that side. Make sure to include the stance position, as the CoP can actually be placed in convex
      // hull of both the reachability region, and the stance position.
      stancePosition.setToZero();
      reachabilityPolygon.set(reachabilityConstraint.getReachabilityPolygonInFootFrame(supportSide));
      reachabilityPolygon.addVertex(stancePosition);
      reachabilityPolygon.update();

      // FIXME instead of translating the reachability polygon, we should probably translate the edge, since that consists of two points, as opposed to however
      // FIXME many points are in the reachability polygon.
      // move the reachability polygon far away from the edge. that prevents the two from intersecting, which you want to avoid to compute the "best" location.
      edgeToExtrude.perpendicular(true, vectorPerpendicularToEdge);
      vectorPerpendicularToEdge.scale(3.0);
      transform.getTranslation().set(edgeToExtrude.midpoint());
      transform.getTranslation().add(vectorPerpendicularToEdge.getX(), vectorPerpendicularToEdge.getY(), 0.0);

      reachabilityPolygon.applyTransform(transform);
      stancePosition.applyTransform(transform);

      // compute the index of the vertex that is the closest. If the vertex belongs to an edge of the reachability polygon that is parallel to the
      // extrusion edge, that needs to be considered.
      int closestIndex = -1;
      int altClosestIndex = -1;
      double closestDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < reachabilityPolygon.getNumberOfVertices(); i++)
      {
         double distance = edgeToExtrude.distanceSquared(reachabilityPolygon.getVertex(i));
         // if this vertex is closer than the closest distance, you know it's either the closest vertex, or one of the closest pair that form the parallel edge
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestIndex = i;
            altClosestIndex = -1;
         }
         // you've already had a vertex at this distance, so this one forms a close parallel edge, and store that index.
         else if (MathTools.epsilonEquals(distance, closestDistance, 1e-4))
         {
            altClosestIndex = i;
         }
      }


      if (altClosestIndex == -1)
      { // The vertex isn't on a parallel edge, so it's just that vertex.
         stepToPack.sub(stancePosition, reachabilityPolygon.getVertex(closestIndex));
      }
      else
      { // The vertex is on a parallel edge, so compute the closest point along the edge to stance.
         EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(stancePosition,
                                                                 reachabilityPolygon.getVertex(closestIndex),
                                                                 reachabilityPolygon.getVertex(altClosestIndex),
                                                                 origin);
         stepToPack.sub(stancePosition, origin);
      }
   }
}
