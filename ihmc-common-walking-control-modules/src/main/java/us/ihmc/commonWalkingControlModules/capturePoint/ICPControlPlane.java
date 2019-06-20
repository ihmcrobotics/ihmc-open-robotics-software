package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
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

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame planarRegionFrame;
   private final FramePoint3D tempProjectedFramePoint = new FramePoint3D();
   private final FramePoint2D tempFramePoint2D = new FramePoint2D();

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();
   private final FramePoint3D intersectionToThrowAway = new FramePoint3D();

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   private final ConvexPolygon2D scaledConvexHull = new ConvexPolygon2D();

   private final double gravityZ;

   public ICPControlPlane(ReferenceFrame centerOfMassFrame, double gravityZ, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.gravityZ = gravityZ;

      controlPlaneHeight = new YoDouble("controlPlaneHeight", registry);
      parentRegistry.addChild(registry);

      planarRegionFrame = new ReferenceFrame("planarRegionFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(planarRegionTransformToWorld);
         }
      };
   }

   public void setOmega0(double omega0)
   {
      double heightOfPlane = -gravityZ / MathTools.square(omega0);
      controlPlaneHeight.set(heightOfPlane);
   }

   public double getControlPlaneHeight()
   {
      return controlPlaneHeight.getDoubleValue();
   }

   public void projectPointOntoControlPlane(ReferenceFrame desiredReferenceFrame, FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack)
   {
      tempFramePoint.setIncludingFrame(pointToProject);
      tempFramePoint.changeFrame(centerOfMassFrame);

      projectPointOntoControlPlane(tempFramePoint, projectionToPack, controlPlaneHeight.getDoubleValue());

      projectionToPack.changeFrame(desiredReferenceFrame);
   }


   public void projectPointFromPlaneOntoSurface(ReferenceFrame desiredReferenceFrame, FramePoint2DReadOnly pointToProject, FramePoint3D projectionToPack, double surfaceHeightInWorld)
   {
      tempFramePoint2D.setIncludingFrame(pointToProject);
      tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);

      tempFramePoint.setIncludingFrame(tempFramePoint2D, surfaceHeightInWorld);
      tempFramePoint.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = tempFramePoint.getZ();
      projectPointFromControlPlaneOntoSurface(tempFramePoint, projectionToPack, controlPlaneHeight.getDoubleValue(), surfaceHeightInCoMFrame);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }


   public void projectPointFromPlaneOntoPlanarRegion(ReferenceFrame desiredReferenceFrame, FramePoint2DReadOnly pointToProject, FramePoint3D projectionToPack, PlanarRegion planarRegion)
   {
      tempFramePoint2D.setIncludingFrame(pointToProject);
      tempFramePoint2D.changeFrameAndProjectToXYPlane(centerOfMassFrame);

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
      convexPolygonInControlPlaneToPack.clear();

      planarRegion.getTransformToWorld(planarRegionTransformToWorld);
      planarRegionFrame.update();

      for (int vertexIndex = 0; vertexIndex < convexHull.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = convexHull.getVertex(vertexIndex);
         tempFramePoint2D.setToZero(planarRegionFrame);
         tempFramePoint2D.set(vertex);
         tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);

         double vertexZ = planarRegion.getPlaneZGivenXY(tempFramePoint2D.getX(), tempFramePoint.getY());

         tempFramePoint.setToZero(worldFrame);
         tempFramePoint.set(tempFramePoint2D, vertexZ);
         tempFramePoint.changeFrame(centerOfMassFrame);

         projectPoint(tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue(), tempFramePoint.getZ());

         tempProjectedFramePoint.changeFrame(worldFrame);

         convexPolygonInControlPlaneToPack.addVertex(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
      convexPolygonInControlPlaneToPack.update();
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2D convexPolygonInControlPlaneToPack, double distanceInside)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      planarRegion.getTransformToWorld(planarRegionTransformToWorld);
      planarRegionFrame.update();

      scaler.scaleConvexPolygon(convexHull, distanceInside, scaledConvexHull);

      for (int vertexIndex = 0; vertexIndex < scaledConvexHull.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = scaledConvexHull.getVertex(vertexIndex);
         tempFramePoint2D.setToZero(planarRegionFrame);
         tempFramePoint2D.set(vertex);
         tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);

         double vertexZ = planarRegion.getPlaneZGivenXY(tempFramePoint2D.getX(), tempFramePoint2D.getY());

         tempFramePoint.setToZero(worldFrame);
         tempFramePoint.set(tempFramePoint2D, vertexZ);
         tempFramePoint.changeFrame(centerOfMassFrame);

         projectPoint(tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue(), tempFramePoint.getZ());

         tempProjectedFramePoint.changeFrame(worldFrame);

         convexPolygonInControlPlaneToPack.addVertex(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
      convexPolygonInControlPlaneToPack.update();
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2D footstepPolygon,
         ConvexPolygon2D convexPolygonInControlPlaneToPack, double distanceInside)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      planarRegion.getTransformToWorld(planarRegionTransformToWorld);
      planarRegionFrame.update();

      scaler.scaleConvexPolygonToContainInteriorPolygon(convexHull, footstepPolygon, distanceInside, scaledConvexHull);

      for (int vertexIndex = 0; vertexIndex < scaledConvexHull.getNumberOfVertices(); vertexIndex++)
      {
         Point2DReadOnly vertex = scaledConvexHull.getVertex(vertexIndex);
         tempFramePoint2D.setToZero(planarRegionFrame);
         tempFramePoint2D.set(vertex);
         tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);

         double vertexZ = planarRegion.getPlaneZGivenXY(tempFramePoint2D.getX(), tempFramePoint2D.getY());

         tempFramePoint.setToZero(worldFrame);
         tempFramePoint.set(tempFramePoint2D, vertexZ);
         tempFramePoint.changeFrame(centerOfMassFrame);

         projectPoint(tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue(), tempFramePoint.getZ());

         tempProjectedFramePoint.changeFrame(worldFrame);

         convexPolygonInControlPlaneToPack.addVertex(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
      convexPolygonInControlPlaneToPack.update();
   }

   private static void projectPointOntoControlPlane(FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack, double planeHeight)
   {
      double unprojectedHeight = pointToProject.getZ();
      projectPoint(pointToProject, projectionToPack, planeHeight, unprojectedHeight);
   }

   private static void projectPointFromControlPlaneOntoSurface(FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack, double planeHeight, double surfaceHeight)
   {
      projectPoint(pointToProject, projectionToPack, surfaceHeight, planeHeight);
   }

   private static void projectPoint(FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack, double projectedHeight, double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }
}
