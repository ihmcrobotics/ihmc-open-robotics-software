package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class CaptureRegionSafetyHeuristics
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameLine2D lineOfMinimalAction = new FrameLine2D();
   private final FrameConvexPolygon2D saferCaptureRegion = new FrameConvexPolygon2D();

   private final YoFrameLine2D yoLineOfMinimalAction = new YoFrameLine2D("yoLineOfMinimalAction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameConvexPolygon2D yoSafetyBiasedCaptureRegion = new YoFrameConvexPolygon2D("safetyBiasedCaptureRegion",
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 30,
                                                                                                 registry);

   private final DoubleParameter distanceToProjectIntoCaptureRegion = new DoubleParameter("distanceToProjectIntoCaptureRegion", registry, 0.08);
   private final DoubleParameter extraDistanceToStepFromStanceFoot = new DoubleParameter("extraDistanceToStepFromStanceFoot", registry, 0.05);

   private final List<FixedFramePoint2DBasics> verticesVisibleFromStance = new ArrayList<>();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FrameVector2D vectorToVertex = new FrameVector2D();
   private final FramePoint2D projectedPoint = new FramePoint2D();

   private final DoubleProvider reachabilityLimit;

   public CaptureRegionSafetyHeuristics(DoubleProvider reachabilityLimit, YoRegistry parentRegistry)
   {
      this(reachabilityLimit, parentRegistry, null);
   }

   public CaptureRegionSafetyHeuristics(DoubleProvider reachabilityLimit, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.reachabilityLimit = reachabilityLimit;

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Safety Biased Capture Region",
                                                                       yoSafetyBiasedCaptureRegion,
                                                                       Color.GREEN,
                                                                       false,
                                                                       true);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);
      }

      parentRegistry.addChild(registry);
   }

   public void computeCaptureRegionWithSafetyHeuristics(FramePoint2DReadOnly capturePoint,
                                                        FramePoint2DReadOnly stancePosition,
                                                        FrameConvexPolygon2DReadOnly oneStepCaptureRegion)
   {
      saferCaptureRegion.setIncludingFrame(oneStepCaptureRegion);

      lineOfMinimalAction.setIncludingFrame(stancePosition, capturePoint);
      lineOfMinimalAction.changeFrame(oneStepCaptureRegion.getReferenceFrame());
      yoLineOfMinimalAction.setMatchingFrame(lineOfMinimalAction);

      this.stancePosition.setIncludingFrame(stancePosition);
      this.stancePosition.changeFrame(oneStepCaptureRegion.getReferenceFrame());

      if (computeVisibiltyOfVerticesFromStance(saferCaptureRegion))
      {
         projectVerticesVisibleToStanceInward();
      }

      projectVerticesFacingTheGoalTowardsTheMiddle();


      saferCaptureRegion.update();

      yoSafetyBiasedCaptureRegion.setMatchingFrame(saferCaptureRegion, false);
   }

   private boolean computeVisibiltyOfVerticesFromStance(FrameConvexPolygon2DBasics oneStepCaptureRegion)
   {
      verticesVisibleFromStance.clear();

      oneStepCaptureRegion.checkReferenceFrameMatch(stancePosition);
      int lineOfSightStartIndex = oneStepCaptureRegion.lineOfSightStartIndex(stancePosition);
      int lineOfSightEndIndex = oneStepCaptureRegion.lineOfSightEndIndex(stancePosition);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;

      while (true)
      {
         verticesVisibleFromStance.add(oneStepCaptureRegion.getVertexUnsafe(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            verticesVisibleFromStance.add(oneStepCaptureRegion.getVertexUnsafe(index));
            break;
         }
      }

      return true;
   }

   private void projectVerticesVisibleToStanceInward()
   {
      vectorToVertex.setReferenceFrame(stancePosition.getReferenceFrame());

      for (int i = 0; i < verticesVisibleFromStance.size(); i++)
      {
         FixedFramePoint2DBasics vertexToProject = verticesVisibleFromStance.get(i);
         vectorToVertex.sub(vertexToProject, stancePosition);

         if (vectorToVertex.length() > reachabilityLimit.getValue() - extraDistanceToStepFromStanceFoot.getValue())
            continue;

         double maxProjectionDistance = findMaximumProjectionDistance(vectorToVertex.length(),
                                                                      reachabilityLimit.getValue(),
                                                                      vectorToVertex.angle(lineOfMinimalAction.getDirection()));

         vertexToProject.scaleAdd(Math.min(maxProjectionDistance, extraDistanceToStepFromStanceFoot.getValue()), lineOfMinimalAction.getDirection(), vertexToProject);
      }
   }

   private static double findMaximumProjectionDistance(double distanceToPointToProject, double maxDistanceToProjectedPoint, double projectionAngle)
   {
      double A = maxDistanceToProjectedPoint;
      double a = Math.PI - Math.abs(projectionAngle);
      double B = distanceToPointToProject;
      double b = Math.asin(B * Math.sin(a) / A);
      double c = Math.PI - b - a;

      return Math.sqrt(A * A + B * B - 2.0 * A * B * Math.cos(c));
   }

   private void projectVerticesFacingTheGoalTowardsTheMiddle()
   {
      projectedPoint.setReferenceFrame(lineOfMinimalAction.getReferenceFrame());

      for (int i = 0; i < saferCaptureRegion.getNumberOfVertices(); i++)
      {
         lineOfMinimalAction.orthogonalProjection(saferCaptureRegion.getVertex(i), projectedPoint);

         FixedFramePoint2DBasics safeVertex = saferCaptureRegion.getVertexUnsafe(i);

         double projectionDistance = safeVertex.distance(projectedPoint);
         if (projectionDistance < distanceToProjectIntoCaptureRegion.getValue())
            safeVertex.set(projectedPoint);
         else
            safeVertex.interpolate(saferCaptureRegion.getVertex(i), projectedPoint, distanceToProjectIntoCaptureRegion.getValue() / projectionDistance);
      }
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegionWithSafetyMargin()
   {
      return saferCaptureRegion;
   }
}
