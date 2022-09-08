package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

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
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 35, registry);

   private final BooleanProvider useCrossOverSteps;

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;

   final FrameVector2D vertexExtrusionVector = new FrameVector2D();
   final FramePoint2D extrudedFirstVertex = new FramePoint2D();
   final FramePoint2D extrudedSecondVertex = new FramePoint2D();
   private final FrameLineSegment2D edgeToExtrude = new FrameLineSegment2D();
   private final FrameVector2D vectorPerpendicularToEdge = new FrameVector2D();

   private MultiStepCaptureRegionVisualizer visualizer = null;

   public MultiStepCaptureRegionCalculator(StepAdjustmentReachabilityConstraint reachabilityConstraint,
                                           BooleanProvider useCrossOverSteps,
                                           YoRegistry parentRegistry)
   {
      this(reachabilityConstraint, useCrossOverSteps, parentRegistry, null);
   }

   public MultiStepCaptureRegionCalculator(StepAdjustmentReachabilityConstraint reachabilityConstraint,
                                           BooleanProvider useCrossOverSteps,
                                           YoRegistry parentRegistry,
                                           YoGraphicsListRegistry graphicsListRegistry)
   {
      this.reachabilityConstraint = reachabilityConstraint;
      this.useCrossOverSteps = useCrossOverSteps;

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Multi Step Capture Region", yoMultiStepRegion, Color.YELLOW, false, false);
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

   private final SideDependentList<ConvexPolygon2D> reachabilityPolygonsWithOrigin = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());

   /**
    * Computes the reachability aware NStep capture region
    *
    * @param currentStanceSide    foot side that is currently on the ground
    * @param oneStepCaptureRegion one-step capture region to expand
    * @param stepDuration         total step duration. This is the sum of the swing and transfer duration.
    * @param omega                time constant of the LIP
    * @param stepsInQueue         number of steps you are allowed to take
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

      for (RobotSide supportSide : RobotSide.values)
      { // Pre-compute the reachability region for getBestStepForSide
         ConvexPolygon2D polygon = reachabilityPolygonsWithOrigin.get(supportSide);
         // compute the reachability polygon for that side. Make sure to include the stance position, as the CoP can actually be placed in convex
         // hull of both the reachability region, and the stance position.
         if (useCrossOverSteps.getValue())
            polygon.set(reachabilityConstraint.getTotalReachabilityHull(supportSide));
         else
            polygon.set(reachabilityConstraint.getReachabilityPolygonInFootFrame(supportSide));
         polygon.addVertex(0.0, 0.0);
         polygon.update();
      }

      for (int i = 0; i < oneStepCaptureRegion.getNumberOfVertices(); i++)
      {
         // compute the current edge of the capture region, which will then be extruded in a certain direction.
         edgeToExtrude.setIncludingFrame(oneStepCaptureRegion.getVertex(i), oneStepCaptureRegion.getNextVertex(i));

         // compute how much additional capturability you get along that edge by considering how additional steps can be taken. These also consider the
         // reachability constraints of the robot during that expansion.
         computeNStepExpansionAlongStep(reachabilityPolygonsWithOrigin,
                                        currentStanceSide,
                                        edgeToExtrude,
                                        vertexExtrusionVector,
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

   private final FrameVector2D bestStepDirectionForStanceSide = new FrameVector2D();
   private final FrameVector2D bestStepDirectionForSwingSide = new FrameVector2D();

   private void computeNStepExpansionAlongStep(SideDependentList<? extends ConvexPolygon2DReadOnly> initialReachabilityRegions,
                                               RobotSide currentStanceSide,
                                               FrameLineSegment2DReadOnly edgeToExtrude,
                                               FrameVector2DBasics expansionToPack,
                                               double currentSupportMultiplier,
                                               double oppositeSupportMultiplier)
   {

      bestStepDirectionForStanceSide.setReferenceFrame(edgeToExtrude.getReferenceFrame());
      bestStepDirectionForSwingSide.setReferenceFrame(edgeToExtrude.getReferenceFrame());

      // Compute the step for each side that would best help recover from additiional error in that direction.
      getDirectionOfFurthestReachableStepFromEdge(initialReachabilityRegions.get(currentStanceSide), edgeToExtrude, bestStepDirectionForStanceSide);
      getDirectionOfFurthestReachableStepFromEdge(initialReachabilityRegions.get(currentStanceSide.getOppositeSide()),
                                                  edgeToExtrude,
                                                  bestStepDirectionForSwingSide);

      // Scale those step lengths based on the exponential time effects of the step durations, which is just up-integrated the LIP dynamics.
      expansionToPack.setAndScale(currentSupportMultiplier, bestStepDirectionForStanceSide);
      expansionToPack.scaleAdd(oppositeSupportMultiplier, bestStepDirectionForSwingSide, expansionToPack);
   }

   private final Point2D origin = new Point2D();
   private final Point2DReadOnly stancePosition = new Point2D();
   private final Point2D tempPoint = new Point2D();
   private final LineSegment2D extrudedEdge = new LineSegment2D();

   private final Vector2D translation = new Vector2D();

   /**
    * Computes the step vector that maximizes the distance away from the capture region the robot can step and still be N-Step capturable. This is the
    * point in the reachability polygon that would first contact the capture region, if it is moved in that direction.
    */
   private void getDirectionOfFurthestReachableStepFromEdge(ConvexPolygon2DReadOnly initialReachabilityRegion,
                                                            FrameLineSegment2DReadOnly edgeToExtrude,
                                                            FrameVector2DBasics stepToPack)
   {
      // move the edge far away from the reachability polygon. that prevents the two from intersecting, which you want to avoid to compute the "best" location.
      edgeToExtrude.perpendicular(true, vectorPerpendicularToEdge);
      edgeToExtrude.midpoint(tempPoint);
      vectorPerpendicularToEdge.negate();
      translation.scaleAdd(3.0, vectorPerpendicularToEdge, tempPoint);

      extrudedEdge.set(edgeToExtrude);
      extrudedEdge.translate(translation);

      // compute the index of the vertex that is the closest. If the vertex belongs to an edge of the reachability polygon that is parallel to the
      // extrusion edge, that needs to be considered.
      int closestIndex = -1;
      int equivalentlyCloseIndex = -1;
      double closestDistanceSquared = Double.POSITIVE_INFINITY;
      for (int i = 0; i < initialReachabilityRegion.getNumberOfVertices(); i++)
      {
         double distanceSquared = extrudedEdge.distanceSquared(initialReachabilityRegion.getVertex(i));
         // if this vertex is closer than the closest distance, you know it's either the closest vertex, or one of the closest pair that form the parallel edge
         if (distanceSquared < closestDistanceSquared)
         {
            closestDistanceSquared = distanceSquared;
            closestIndex = i;
            equivalentlyCloseIndex = -1;
         }
         // you've already had a vertex at this distance, so this one forms a close parallel edge, and store that index.
         else if (MathTools.epsilonEquals(distanceSquared, closestDistanceSquared, 1e-4))
         {
            equivalentlyCloseIndex = i;
         }
      }

      if (equivalentlyCloseIndex == -1)
      { // The vertex isn't on a parallel edge, so it's just that vertex.
         stepToPack.setAndNegate(initialReachabilityRegion.getVertex(closestIndex));
      }
      else
      { // The vertex is on a parallel edge, so compute the closest point along the edge to stance.
         EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(stancePosition,
                                                                 initialReachabilityRegion.getVertex(closestIndex),
                                                                 initialReachabilityRegion.getVertex(equivalentlyCloseIndex),
                                                                 origin);
         stepToPack.setAndNegate(origin);
      }
   }
}
