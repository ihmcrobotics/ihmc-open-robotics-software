package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SmartCMPProjector extends CMPProjector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // local points so changing their frame will not modify the objects passed in
   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FramePoint2d projectedCMP = new FramePoint2d();
   private final FrameConvexPolygon2d projectionArea = new FrameConvexPolygon2d();
   private final FramePoint2d capturePoint = new FramePoint2d();
   private final FramePoint2d finalCapturePoint = new FramePoint2d();

   // visualization
   private final YoFramePoint2d yoDesiredCMP = new YoFramePoint2d("DesiredCMP", worldFrame, registry);
   private final YoFramePoint2d yoProjectedCMP = new YoFramePoint2d("ProjectedCMP", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoProjectionArea = new YoFrameConvexPolygon2d("CMPProjectionArea", worldFrame, 10, registry);

   // debugging and state of the projector
   private final BooleanYoVariable cmpWasProjected = new BooleanYoVariable("CmpWasProjected", registry);

   // temporary variables to avoid garbage generation
   private final BoundingBox2d tempBoundingBox = new BoundingBox2d();
   private final FrameLine2d icpToCMPLine = new FrameLine2d();
   private final FramePoint2d intersection1 = new FramePoint2d();
   private final FramePoint2d intersection2 = new FramePoint2d();
   private final FramePoint2d vertex = new FramePoint2d();
   private final FrameVector2d icpToCMPVector = new FrameVector2d();
   private final FrameVector2d icpToCandidateVector = new FrameVector2d();
   private final FrameLine2d finalICPToICPLine = new FrameLine2d();
   private final FrameVector2d finalICPToICPVector = new FrameVector2d();
   private final FramePoint2d centroid = new FramePoint2d();

   // for debugging and to check what method was used
   public enum ProjectionMethod {
      NONE,
      SMALL_AREA_CENTROID,
      RAY_THROUGH_AREA,
      TOWARDS_FINAL,
      TOWARDS_FINAL_MIN_ANGLE,
      ORTHOGONAL_PROJECTION
   }
   private final EnumYoVariable<ProjectionMethod> activeProjection = new EnumYoVariable<>("ActiveCMPProjection", registry, ProjectionMethod.class);

   public SmartCMPProjector(YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      activeProjection.set(ProjectionMethod.NONE);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
      {
         YoArtifactPosition desiredCMPViz = new YoArtifactPosition("Desired CMP Position", yoDesiredCMP, GraphicType.SOLID_BALL, DarkRed().getAwtColor(),
               0.008);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), desiredCMPViz);

         YoArtifactPosition projectedCMPViz = new YoArtifactPosition("Projected CMP Position", yoProjectedCMP, GraphicType.BALL_WITH_ROTATED_CROSS,
               DarkRed().getAwtColor(), 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), projectedCMPViz);

         YoArtifactPolygon projectionAreaViz = new YoArtifactPolygon("CMP Projection Area", yoProjectionArea, Blue().getAwtColor(), false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), projectionAreaViz);
      }
   }

   public void projectCMP(FramePoint2d desiredCMP, FrameConvexPolygon2d projectionArea, FramePoint2d capturePoint, FramePoint2d finalCapturePoint)
   {
      // store arguments in local variables and change the frames to match the projection area
      this.projectionArea.setIncludingFrameAndUpdate(projectionArea);
      this.desiredCMP.setIncludingFrame(desiredCMP);
      this.desiredCMP.changeFrameAndProjectToXYPlane(projectionArea.getReferenceFrame());
      this.capturePoint.setIncludingFrame(capturePoint);
      this.capturePoint.changeFrameAndProjectToXYPlane(projectionArea.getReferenceFrame());
      if (finalCapturePoint != null)
         this.finalCapturePoint.setIncludingFrame(finalCapturePoint);
      else
         this.finalCapturePoint.setToNaN();
      this.finalCapturePoint.changeFrameAndProjectToXYPlane(projectionArea.getReferenceFrame());

      // do the projection
      projectCMP();
      cmpWasProjected.set(!activeProjection.valueEquals(ProjectionMethod.NONE));

      // set the desired CMP to the projection
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();
      desiredCMP.setIncludingFrame(projectedCMP);
      desiredCMP.changeFrame(returnFrame);

      // visualize
      this.projectionArea.changeFrameAndProjectToXYPlane(worldFrame);
      this.desiredCMP.changeFrameAndProjectToXYPlane(worldFrame);
      projectedCMP.changeFrameAndProjectToXYPlane(worldFrame);
      if (cmpWasProjected.getBooleanValue())
      {
         yoProjectionArea.setFrameConvexPolygon2d(this.projectionArea);
         yoDesiredCMP.set(this.desiredCMP);
         yoProjectedCMP.set(projectedCMP);
      }
      else
      {
         yoProjectionArea.setFrameConvexPolygon2d(null);
         yoDesiredCMP.setToNaN();
         yoProjectedCMP.setToNaN();
      }
   }

   private void projectCMP()
   {
      // if the desired CMP is inside the support do nothing
      if (projectionArea.isPointInside(desiredCMP))
      {
         projectedCMP.setIncludingFrame(desiredCMP);
         activeProjection.set(ProjectionMethod.NONE);
         return;
      }

      // if the support area is small set the desired CMP to centroid
      projectionArea.getBoundingBox(tempBoundingBox);
      if (tempBoundingBox.getDiagonalLengthSquared() < 0.01 * 0.01)
      {
         projectionArea.getCentroid(projectedCMP);
         activeProjection.set(ProjectionMethod.SMALL_AREA_CENTROID);
         return;
      }

      // if the ICP is just on the edge move it out a little bit
      if (projectionArea.distance(capturePoint) < 1.0e-6)
      {
         projectionArea.getCentroid(centroid);
         centroid.sub(capturePoint);
         centroid.scale(1.0e-6);
         capturePoint.sub(centroid);
      }

      // if a ray (!) from ICP through the CMP intersects the support chose the intersection closest to the desired CMP
      icpToCMPLine.setIncludingFrame(capturePoint, desiredCMP);
      if (projectionArea.intersectionWithRay(icpToCMPLine, intersection1, intersection2) > 0)
      {
         FramePoint2d candidate = closestPoint(desiredCMP, intersection1, intersection2);
         icpToCMPVector.setToZero(projectionArea.getReferenceFrame());
         icpToCMPVector.sub(desiredCMP, capturePoint);
         icpToCandidateVector.setToZero(projectionArea.getReferenceFrame());
         icpToCandidateVector.sub(candidate, capturePoint);

         // make sure the CMP does not get projected too far from its original position
         double maxICPSpeedIncreaseFactor = 3.0;
         double desiredDistance = icpToCMPVector.length();
         double distanceAfterProjecting = icpToCandidateVector.length();
         boolean projectionClose = distanceAfterProjecting < maxICPSpeedIncreaseFactor * desiredDistance;

         // make sure the ICP is pushed in the right direction
         double angle = icpToCMPVector.angle(icpToCandidateVector);
         if (angle < 1.0e-7 && projectionClose)
         {
            projectedCMP.setIncludingFrame(candidate);
            activeProjection.set(ProjectionMethod.RAY_THROUGH_AREA);
            return;
         }
      }

      // if we have a final desired ICP check if we can push the ICP towards that point as slow as possible
      if (!finalCapturePoint.containsNaN())
      {
         finalICPToICPLine.setIncludingFrame(finalCapturePoint, capturePoint);
         if (projectionArea.intersectionWithRay(finalICPToICPLine, intersection1, intersection2) > 0)
         {
            FramePoint2d candidate = closestPoint(capturePoint, intersection1, intersection2);
            finalICPToICPVector.setToZero(projectionArea.getReferenceFrame());
            finalICPToICPVector.sub(capturePoint, finalCapturePoint);
            icpToCandidateVector.setToZero(projectionArea.getReferenceFrame());
            icpToCandidateVector.sub(candidate, capturePoint);

            // make sure the ICP is pushed in the right direction
            double angle = finalICPToICPVector.angle(icpToCandidateVector);
            if (angle < 1.0e-7)
            {
               projectedCMP.setIncludingFrame(candidate);
               activeProjection.set(ProjectionMethod.TOWARDS_FINAL);
               return;
            }
         }
      }

      // if we have a final desired ICP find the vertex on the projection area that achieves the movement toward
      // the final ICP best.
      if (!finalCapturePoint.containsNaN())
      {
         double angle = Double.POSITIVE_INFINITY;
         for (int i = 0; i < projectionArea.getNumberOfVertices(); i++)
         {
            vertex.setIncludingFrame(projectionArea.getReferenceFrame(), projectionArea.getVertex(i));
            finalICPToICPVector.setToZero(projectionArea.getReferenceFrame());
            finalICPToICPVector.sub(capturePoint, finalCapturePoint);
            icpToCandidateVector.setToZero(projectionArea.getReferenceFrame());
            icpToCandidateVector.sub(vertex, capturePoint);

            double newAngle = icpToCandidateVector.angle(finalICPToICPVector);
            if (newAngle < angle)
            {
               angle = newAngle;
               projectedCMP.setIncludingFrame(vertex);
            }
         }
         // only do this if it actually helps (pushed the ICP in the right direction)
         if (angle < Math.PI / 4.0)
         {
            activeProjection.set(ProjectionMethod.TOWARDS_FINAL_MIN_ANGLE);
            return;
         }
      }

      // as a last resort do an orthogonal projection of the desired CMP onto the projection area
      projectedCMP.setIncludingFrame(desiredCMP);
      projectionArea.orthogonalProjection(projectedCMP);
      activeProjection.set(ProjectionMethod.ORTHOGONAL_PROJECTION);
   }

   private FramePoint2d closestPoint(FramePoint2d point, FramePoint2d candidate1, FramePoint2d candidate2)
   {
      if (candidate1.containsNaN() && candidate2.containsNaN())
         return null;
      if (candidate1.containsNaN())
         return candidate2;
      if (candidate2.containsNaN())
         return candidate1;
      if (point.distance(candidate1) <= point.distance(candidate2))
         return candidate1;
      return candidate2;
   }

   public ProjectionMethod getProjectionMethod()
   {
      return activeProjection.getEnumValue();
   }

   @Override
   public boolean getWasCMPProjected()
   {
      return cmpWasProjected.getBooleanValue();
   }

   @Override
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2d finalDesiredCapturePoint,
         FramePoint2d desiredCMP)
   {
      projectCMP(desiredCMP, supportPolygon, capturePoint, finalDesiredCapturePoint);
   }

}
