package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import org.fxyz3d.geometry.Vector3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FrameScalableBoundingBox3d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ICPControlPlane
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble controlPlaneHeight;
   private final ReferenceFrame centerOfMassFrame;

   public ICPControlPlane(YoDouble omega0, ReferenceFrame centerOfMassFrame, double gravityZ, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      controlPlaneHeight = new YoDouble("controlPlaneHeight", registry);
      parentRegistry.addChild(registry);

      omega0.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double heightOfPlane = -gravityZ / Math.pow(omega0.getDoubleValue(), 2.0);
            controlPlaneHeight.set(heightOfPlane);
         }
      });
   }

   public double getControlPlaneHeight()
   {
      return controlPlaneHeight.getDoubleValue();
   }

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempProjectedFramePoint = new FramePoint3D();
   private final FramePoint2D tempFramePoint2D = new FramePoint2D();

   public void projectPointOntoControlPlane(ReferenceFrame desiredReferenceFrame, FramePoint3D pointToProject, FramePoint3D projectionToPack)
   {
      tempFramePoint.setIncludingFrame(pointToProject);
      tempFramePoint.changeFrame(centerOfMassFrame);

      projectPointOntoControlPlane(tempFramePoint, projectionToPack, controlPlaneHeight.getDoubleValue());

      projectionToPack.changeFrame(desiredReferenceFrame);
   }


   public void projectPointFromPlaneOntoSurface(ReferenceFrame desiredReferenceFrame, FramePoint2D pointToProject, FramePoint3D projectionToPack, double surfaceHeightInWorld)
   {
      tempFramePoint2D.setIncludingFrame(pointToProject);
      tempFramePoint2D.changeFrame(worldFrame);

      tempFramePoint.setIncludingFrame(tempFramePoint2D, surfaceHeightInWorld);
      tempFramePoint.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = tempFramePoint.getZ();
      projectPointFromControlPlaneOntoSurface(tempFramePoint, projectionToPack, controlPlaneHeight.getDoubleValue(), surfaceHeightInCoMFrame);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }


   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();
   private final FramePoint3D intersectionToThrowAway = new FramePoint3D();

   public void projectPointFromPlaneOntoPlanarRegion(ReferenceFrame desiredReferenceFrame, FramePoint2D pointToProject, FramePoint3D projectionToPack, PlanarRegion planarRegion)
   {
      tempFramePoint2D.setIncludingFrame(pointToProject);
      tempFramePoint2D.changeFrame(centerOfMassFrame);

      tempFramePoint.setIncludingFrame(tempFramePoint2D, controlPlaneHeight.getDoubleValue());
      tempFramePoint.changeFrame(worldFrame);

      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      rayDirection.set(tempFramePoint);
      rayDirection.sub(centerOfMassPosition);

      projectionToPack.setToZero(worldFrame);

      BoundingBox3D boundingBoxInWorld = planarRegion.getBoundingBox3dInWorld();
      boundingBoxInWorld.intersectionWithRay3D(centerOfMassPosition, rayDirection, projectionToPack, intersectionToThrowAway);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   public void projectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2D convexPolygonInControlPlaneToPack)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();

      for (int vertexIndex = 0; vertexIndex < convexHull.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = convexHull.getVertex(vertexIndex);
         double vertexZ = planarRegion.getPlaneZGivenXY(vertex.getX(), vertex.getY());

         tempFramePoint.setToZero(worldFrame);
         tempFramePoint.set(vertex, vertexZ);
         tempFramePoint.changeFrame(centerOfMassFrame);

         projectPoint(tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue(), tempFramePoint.getZ());

         tempProjectedFramePoint.changeFrame(worldFrame);

         convexPolygonInControlPlaneToPack.addVertex(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
   }

   private static void projectPointOntoControlPlane(FramePoint3D pointToProject, FramePoint3D projectionToPack, double planeHeight)
   {
      double unprojectedHeight = pointToProject.getZ();
      projectPoint(pointToProject, projectionToPack, planeHeight, unprojectedHeight);
   }

   private static void projectPointFromControlPlaneOntoSurface(FramePoint3D pointToProject, FramePoint3D projectionToPack, double planeHeight, double surfaceHeight)
   {
      projectPoint(pointToProject, projectionToPack, surfaceHeight, planeHeight);
   }

   private static void projectPoint(FramePoint3D pointToProject, FramePoint3D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }
}
