package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.*;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
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
public class MultiStepCaptureRegionCalculator implements SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger stepsInQueue = new YoInteger("stepsInQueue", registry);
   private final YoInteger stepsConsideringForRecovery = new YoInteger("stepsConsideringForRecovery", registry);
   private final IntegerParameter maxStepsToConsider = new IntegerParameter("maxStepsToConsiderForRecovery", registry, 10);

   private final FrameConvexPolygon2D multiStepRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D regionToExpand = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 75, registry);

   private final BooleanProvider useCrossOverSteps;

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;

   private final RecyclingArrayList<FramePoint2DBasics> expansionPoints = new RecyclingArrayList<>(FramePoint2D::new);

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
         throw new RuntimeException("Doesn't work for counter clockwise yet");

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

      regionToExpand.setIncludingFrame(oneStepCaptureRegion);
      multiStepRegion.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      // perform the expansion to get the second step capture region.
      if (stepsConsideringForRecovery.getIntegerValue() > 0)
      {
         expandCaptureRegion(regionToExpand,
                             reachabilityPolygonsWithOrigin.get(currentStanceSide.getOppositeSide()),
                             multiStepRegion,
                             oppositeSupportMultiplier);
      }
      else
      {
         multiStepRegion.set(regionToExpand);
      }
      // perform an additional expansion to get the three step (or higher) capture region.
      if (stepsConsideringForRecovery.getIntegerValue() > 1)
      {
         regionToExpand.set(multiStepRegion);
         expandCaptureRegion(regionToExpand, reachabilityPolygonsWithOrigin.get(currentStanceSide), multiStepRegion, currentSupportMultiplier);
      }

      yoMultiStepRegion.setMatchingFrame(multiStepRegion, false);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoMultiStepRegion;
   }

   private void expandCaptureRegion(FrameConvexPolygon2DReadOnly regionToExpand,
                                    ConvexPolygon2DReadOnly reachabilityPolygon,
                                    FrameConvexPolygon2DBasics expandedRegionToPack,
                                    double stepMultiplier)
   {
      if (regionToExpand.getNumberOfVertices() > 2)
      {
         expandCaptureRegionPolygon(regionToExpand, reachabilityPolygon, expandedRegionToPack, stepMultiplier);
      }
      else if (regionToExpand.getNumberOfVertices() == 2)
      {
         expandCaptureRegionLine(regionToExpand, reachabilityPolygon, expandedRegionToPack, stepMultiplier);
      }
      else
      {
         expandCaptureRegionPoint(regionToExpand, reachabilityPolygon, expandedRegionToPack, stepMultiplier);
      }
   }

   private void expandCaptureRegionPoint(FrameConvexPolygon2DReadOnly regionToExpand,
                                         ConvexPolygon2DReadOnly reachabilityPolygon,
                                         FrameConvexPolygon2DBasics expandedRegionToPack,
                                         double stepMultiplier)
   {
      expandedRegionToPack.clear();
      FramePoint2DReadOnly vertex = regionToExpand.getVertex(0);

      expandPoint(vertex, reachabilityPolygon, expansionPoints, stepMultiplier);

      for (int expandedPointIndex = 0; expandedPointIndex < expansionPoints.size(); expandedPointIndex++)
      {
         Point2DBasics expansionPoint = expansionPoints.get(expandedPointIndex);
         expandedRegionToPack.addVertex(expansionPoint.getX(), expansionPoint.getY());
      }
      expandedRegionToPack.update();
   }

   private void expandCaptureRegionLine(FrameConvexPolygon2DReadOnly regionToExpand,
                                        ConvexPolygon2DReadOnly reachabilityPolygon,
                                        FrameConvexPolygon2DBasics expandedRegionToPack,
                                        double stepMultiplier)
   {
      expandedRegionToPack.clear();

      for (int vertexIdx = 0; vertexIdx < regionToExpand.getNumberOfVertices(); vertexIdx++)
      {
         FramePoint2DReadOnly vertex = regionToExpand.getVertex(vertexIdx);

         expandLine(vertex, regionToExpand.getNextVertex(vertexIdx), reachabilityPolygon, expansionPoints, stepMultiplier);

         for (int expandedPointIndex = 0; expandedPointIndex < expansionPoints.size(); expandedPointIndex++)
         {
            Point2DBasics expansionPoint = expansionPoints.get(expandedPointIndex);
            expandedRegionToPack.addVertex(expansionPoint.getX(), expansionPoint.getY());
         }
      }
      expandedRegionToPack.update();
   }

   private void expandCaptureRegionPolygon(FrameConvexPolygon2DReadOnly regionToExpand,
                                           ConvexPolygon2DReadOnly reachabilityPolygon,
                                           FrameConvexPolygon2DBasics expandedRegionToPack,
                                           double stepMultiplier)
   {
      expandedRegionToPack.clear();
      FramePoint2DReadOnly vertex = regionToExpand.getVertex(0);
      FramePoint2DReadOnly previousVertex = regionToExpand.getPreviousVertex(0);

      for (int vertexIndex = 0; vertexIndex < regionToExpand.getNumberOfVertices(); vertexIndex++)
      {
         FramePoint2DReadOnly nextVertex = regionToExpand.getNextVertex(vertexIndex);

         expandCorner(previousVertex, vertex, nextVertex, reachabilityPolygon, expansionPoints, stepMultiplier);

         for (int expandedPointIndex = 0; expandedPointIndex < expansionPoints.size(); expandedPointIndex++)
         {
            Point2DBasics expansionPoint = expansionPoints.get(expandedPointIndex);
            expandedRegionToPack.addVertex(expansionPoint.getX(), expansionPoint.getY());
         }

         previousVertex = vertex;
         vertex = nextVertex;
      }
      expandedRegionToPack.update();
   }

   private static void expandPoint(FramePoint2DReadOnly pointToExpand,
                                   ConvexPolygon2DReadOnly reachabilityRegion,
                                   RecyclingArrayList<FramePoint2DBasics> expansionPointsToPack,
                                   double expansionScalar)
   {
      expansionPointsToPack.clear();
      ReferenceFrame referenceFrame = pointToExpand.getReferenceFrame();

      for (int i = 0; i < reachabilityRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = reachabilityRegion.getNextVertex(i);

         FramePoint2DBasics expansionPoint = expansionPointsToPack.add();
         expansionPoint.setReferenceFrame(referenceFrame);
         expansionPoint.scaleAdd(-expansionScalar, vertex, pointToExpand);
      }

      if (expansionPointsToPack.size() == 0)
         expansionPointsToPack.add().set(pointToExpand);
   }

   private static void expandLine(FramePoint2DReadOnly pointToExpand,
                                  FramePoint2DReadOnly otherEnd,
                                  ConvexPolygon2DReadOnly reachabilityRegion,
                                  RecyclingArrayList<FramePoint2DBasics> expansionPointsToPack,
                                  double expansionScalar)
   {
      expansionPointsToPack.clear();

      ReferenceFrame referenceFrame = pointToExpand.getReferenceFrame();

      Point2DReadOnly vertex = reachabilityRegion.getVertex(0);
      Point2DReadOnly precedingPointB = reachabilityRegion.getPreviousVertex(0);

      for (int i = 0; i < reachabilityRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly succeedingPointB = reachabilityRegion.getNextVertex(i);

         // Filter out the point if it woudl end up being an interior point.
         if (!isRayPointingToTheInside(precedingPointB, vertex, succeedingPointB, otherEnd.getX() - pointToExpand.getX(), otherEnd.getY() - pointToExpand.getY()))
         {
            FramePoint2DBasics expansionPoint = expansionPointsToPack.add();
            expansionPoint.setReferenceFrame(referenceFrame);
            expansionPoint.scaleAdd(-expansionScalar, vertex, pointToExpand);
         }

         precedingPointB = vertex;
         vertex = succeedingPointB;
      }

      if (expansionPointsToPack.size() == 0)
         expansionPointsToPack.add().set(pointToExpand);
   }

   private static void expandCorner(FramePoint2DReadOnly precedingPoint,
                                    FramePoint2DReadOnly cornerToExpand,
                                    FramePoint2DReadOnly succeedingPoint,
                                    ConvexPolygon2DReadOnly reachabilityRegion,
                                    RecyclingArrayList<FramePoint2DBasics> expansionPointsToPack,
                                    double expansionScalar)
   {
      expansionPointsToPack.clear();

      ReferenceFrame referenceFrame = precedingPoint.getReferenceFrame();

      Point2DReadOnly vertex = reachabilityRegion.getVertex(0);
      Point2DReadOnly precedingPointB = reachabilityRegion.getPreviousVertex(0);

      for (int i = 0; i < reachabilityRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly succeedingPointB = reachabilityRegion.getNextVertex(i);

         // Filter this reachability vertex if it would end up resulting in an interior point of the expanded region.
         if (isPointASharedNonIntersectingVertex(precedingPoint, cornerToExpand, succeedingPoint, precedingPointB, vertex, succeedingPointB))
         {
            FramePoint2DBasics expansionPoint = expansionPointsToPack.add();
            expansionPoint.setReferenceFrame(referenceFrame);
            expansionPoint.scaleAdd(-expansionScalar, vertex, cornerToExpand);
         }

         precedingPointB = vertex;
         vertex = succeedingPointB;
      }

      if (expansionPointsToPack.size() == 0)
         expansionPointsToPack.add().set(cornerToExpand);
   }

   static boolean isRayPointingToTheInside(Point2DReadOnly precedingPointA,
                                           Point2DReadOnly cornerToCheckA,
                                           Point2DReadOnly succeedingPointA,
                                           Point2DReadOnly sharedPointB,
                                           Point2DReadOnly otherPointOnB)
   {
      return isRayPointingToTheInside(precedingPointA, cornerToCheckA, succeedingPointA,
                                      otherPointOnB.getX() - sharedPointB.getX(),
                                      otherPointOnB.getY() - sharedPointB.getY());
   }

   static boolean isRayPointingToTheInside(Point2DReadOnly precedingPointA,
                                           Point2DReadOnly cornerToCheckA,
                                           Point2DReadOnly succeedingPointA,
                                           double dx,
                                           double dy)
   {
      return isRayPointingToTheInside(precedingPointA.getX() - cornerToCheckA.getX(),
                                      precedingPointA.getY() - cornerToCheckA.getY(),
                                      succeedingPointA.getX() - cornerToCheckA.getX(),
                                      succeedingPointA.getY() - cornerToCheckA.getY(),
                                      dx,
                                      dy);
   }

   private static boolean isRayPointingToTheInside(double dxAPrev, double dyAPrev, double dxANext, double dyANext, double dxB, double dyB)
   {
      boolean isBLeftOfANext = cross(dxANext, dyANext, dxB, dxB) > 0.0;

      if (isBLeftOfANext)
      {
         return false;
      }

      boolean isBLeftOfAPrevious = cross(dxAPrev, dyAPrev, dxB, dyB) >= 0.0;

      return isBLeftOfAPrevious;
   }

   /**
    * Checks to see if two corners of two separate convex polygons result in overlap between the two polygons. Both polygons are assumed to be clockwise
    * ordered.
    *
    * @param precedingPointA
    * @param cornerToCheckA   Intersecting vertex on polygon A
    * @param succeedingPointA
    * @param precedingPointB
    * @param cornerToCheckB   Intersecting vertex on polygon B
    * @param succeedingPointB
    * @return
    */
   static boolean isPointASharedNonIntersectingVertex(Point2DReadOnly precedingPointA,
                                                      Point2DReadOnly cornerToCheckA,
                                                      Point2DReadOnly succeedingPointA,
                                                      Point2DReadOnly precedingPointB,
                                                      Point2DReadOnly cornerToCheckB,
                                                      Point2DReadOnly succeedingPointB)
   {
      return isPointASharedNonIntersectingVertex(precedingPointA.getX() - cornerToCheckA.getX(),
                                                 precedingPointA.getY() - cornerToCheckA.getY(),
                                                 succeedingPointA.getX() - cornerToCheckA.getX(),
                                                 succeedingPointA.getY() - cornerToCheckA.getY(),
                                                 precedingPointB.getX() - cornerToCheckB.getX(),
                                                 precedingPointB.getY() - cornerToCheckB.getY(),
                                                 succeedingPointB.getX() - cornerToCheckB.getX(),
                                                 succeedingPointB.getY() - cornerToCheckB.getY());
   }

   static boolean isPointASharedNonIntersectingVertex(double dxAPrev,
                                                      double dyAPrev,
                                                      double dxANext,
                                                      double dyANext,
                                                      double dxBPrev,
                                                      double dyBPrev,
                                                      double dxBNext,
                                                      double dyBNext)
   {
      boolean isBPreviousLeftOfANext = cross(dxANext, dyANext, dxBPrev, dyBPrev) < 0.0;

      if (isBPreviousLeftOfANext)
      {
         return false;
      }

      boolean isBNextRightOfAPrevious = cross(dxAPrev, dyAPrev, dxBNext, dyBNext) <= 0.0;

      return isBNextRightOfAPrevious;
   }

   private static double cross(double dxA, double dyA, double dxB, double dyB)
   {
      return dxA * dyB - dxB * dyA;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Multi Step Capture Region", yoMultiStepRegion, ColorDefinitions.Yellow()));
      return group;
   }
}
