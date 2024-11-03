package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;

/**
 * This class computes the N-Step capture region, but it does so using the reachability constraint assuming just a maximum reach.
 */
public class SimpleMultiStepCaptureRegionCalculator implements SCS2YoGraphicHolder
{
   private static final int expansionPointsPerCorner = 20;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger stepsInQueue = new YoInteger("stepsInQueue", registry);
   private final YoInteger stepsConsideringForRecovery = new YoInteger("stepsConsideringForRecovery", registry);
   private final IntegerParameter maxStepsToConsider = new IntegerParameter("maxStepsToConsiderForRecovery", registry, 10);

   private final FrameConvexPolygon2D multiStepRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoMultiStepRegion = new YoFrameConvexPolygon2D("multiStepCaptureRegion", ReferenceFrame.getWorldFrame(), 100, registry);

   private final RecyclingArrayList<FramePoint2DBasics> expansionPoints = new RecyclingArrayList<>(FramePoint2D::new);


   public SimpleMultiStepCaptureRegionCalculator(YoRegistry parentRegistry)
   {
      this(parentRegistry, null);
   }

   public SimpleMultiStepCaptureRegionCalculator(YoRegistry parentRegistry,
                                                 YoGraphicsListRegistry graphicsListRegistry)
   {
      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Multi Step Capture Region", yoMultiStepRegion, Color.YELLOW, false, false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);
      }

      parentRegistry.addChild(registry);
   }


   public void reset()
   {
      yoMultiStepRegion.clear();
   }

   /**
    * Computes the reachability aware NStep capture region
    *
    * @param oneStepCaptureRegion one-step capture region to expand
    * @param stepDuration         total step duration. This is the sum of the swing and transfer duration.
    * @param omega                time constant of the LIP
    * @param stepsInQueue         number of steps you are allowed to take
    */
   public void compute(FrameConvexPolygon2DReadOnly oneStepCaptureRegion, double stepDuration, double omega, double stepReach, int stepsInQueue)
   {
      stepsConsideringForRecovery.set(Math.min(stepsInQueue, maxStepsToConsider.getValue()));
      this.stepsInQueue.set(stepsInQueue);

      // Compute the time scaling effects of the future steps.
      double stepExponential = Math.exp(-omega * stepDuration);

      // Compute the maximum effect that stepping with each side will have on increasing the capture region, normalized by length. This is just a
      // concatenation of taking all of those steps.
      double stepMultiplier = stepExponential;
      for (int step = 2; step < stepsInQueue; step++)
         stepMultiplier *= (1 + stepExponential);

      double distance = 0.0;
      for (int step = 1; step < stepsInQueue; step++)
      {
         distance = (distance + stepReach) * stepExponential;
      }
      double bound = stepReach * stepExponential / (1.0 - stepExponential);

      if (!oneStepCaptureRegion.isClockwiseOrdered())
         throw new RuntimeException("Doesn't work for counter clockwise yet");

      multiStepRegion.setReferenceFrame(oneStepCaptureRegion.getReferenceFrame());

      expandCaptureRegion(oneStepCaptureRegion,
                          multiStepRegion,
                          distance);

      yoMultiStepRegion.setMatchingFrame(multiStepRegion, false);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoMultiStepRegion;
   }

   private void expandCaptureRegion(FrameConvexPolygon2DReadOnly regionToExpand,
                                    FrameConvexPolygon2DBasics expandedRegionToPack,
                                    double expansionDistance)
   {
      expandedRegionToPack.clear();
      FramePoint2DReadOnly vertex = regionToExpand.getVertex(0);
      FramePoint2DReadOnly previousVertex = regionToExpand.getPreviousVertex(0);

      for (int vertexIndex = 0; vertexIndex < regionToExpand.getNumberOfVertices(); vertexIndex++)
      {
         FramePoint2DReadOnly nextVertex = regionToExpand.getNextVertex(vertexIndex);

         expandCorner(previousVertex, vertex, nextVertex, expansionPoints, expansionDistance);

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

   private final FrameVector2D startDirection = new FrameVector2D();
   private final FrameVector2D endDirection = new FrameVector2D();

   private void expandCorner(FramePoint2DReadOnly precedingPoint,
                             FramePoint2DReadOnly cornerToExpand,
                             FramePoint2DReadOnly succeedingPoint,
                             RecyclingArrayList<FramePoint2DBasics> expansionDirectionsToPack,
                             double expansionDistance)
   {
      expansionDirectionsToPack.clear();

      startDirection.sub(precedingPoint, cornerToExpand);
      endDirection.sub(succeedingPoint, cornerToExpand);

      double angleBetweenDirections = startDirection.angle(endDirection);
//      if (startDirection.cross(endDirection) < 0.0)
         angleBetweenDirections += Math.PI;

      for (int i = 0; i < expansionPointsPerCorner; i++)
      {
         double alpha = ((double) i) / (expansionPointsPerCorner);

         getPointBetweenVectorsAtDistanceFromOriginCircular(startDirection,
                                                            endDirection,
                                                            alpha,
                                                            angleBetweenDirections,
                                                            expansionDistance,
                                                            cornerToExpand,
                                                            expansionDirectionsToPack.add());
      }

      if (expansionDirectionsToPack.size() == 0)
         expansionDirectionsToPack.add().set(cornerToExpand);
   }

   private final FrameVector2D rotatedFromA = new FrameVector2D();

   public void getPointBetweenVectorsAtDistanceFromOriginCircular(FrameVector2DReadOnly directionA,
                                                                  FrameVector2DReadOnly directionB,
                                                                  double alpha,
                                                                  double angleBetweenDirections,
                                                                  double radius,
                                                                  FramePoint2DReadOnly centerOfCircle,
                                                                  FramePoint2DBasics pointToPack)
   {
      directionA.checkReferenceFrameMatch(directionB.getReferenceFrame());
      directionA.checkReferenceFrameMatch(centerOfCircle.getReferenceFrame());
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      double angleBetweenDirectionsToSetLine = angleBetweenDirections * alpha;

      rotatedFromA.setReferenceFrame(directionA.getReferenceFrame());
      RotationMatrixTools.applyYawRotation(angleBetweenDirectionsToSetLine, directionA, rotatedFromA);

      rotatedFromA.scale(radius / rotatedFromA.norm());

      pointToPack.setIncludingFrame(rotatedFromA);
      pointToPack.add(centerOfCircle);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Multi Step Capture Region", yoMultiStepRegion, ColorDefinitions.Yellow()));
      return group;
   }
}
