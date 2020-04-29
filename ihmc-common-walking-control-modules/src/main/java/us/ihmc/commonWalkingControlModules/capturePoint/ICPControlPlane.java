package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ICPControlPlane
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble controlPlaneHeight;
   private final ReferenceFrame centerOfMassFrame;

   private final RecyclingArrayList<Point3D> convexHullInWorld = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point2DBasics> pointsInPlane = new RecyclingArrayList<>(Point2D::new);

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempProjectedFramePoint = new FramePoint3D();

   private final FramePoint3D planeOrigin = new FramePoint3D();
   private final FrameVector3D planeNormal = new FrameVector3D();

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   private final ConvexPolygon2D scaledConvexHull = new ConvexPolygon2D();

   private final double gravityZ;

   public ICPControlPlane(ReferenceFrame centerOfMassFrame, double gravityZ, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.gravityZ = gravityZ;

      controlPlaneHeight = new YoDouble("controlPlaneHeight", registry);
      parentRegistry.addChild(registry);
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
      projectPointThroughReferenceFrame(centerOfMassFrame, pointToProject, projectionToPack, controlPlaneHeight.getDoubleValue());

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   // FIXME clean up?
   public void projectPointFromControlPlaneOntoSurface(ReferenceFrame desiredReferenceFrame,
                                                       FramePoint2DReadOnly pointToProject,
                                                       FramePoint3DBasics projectionToPack,
                                                       double surfaceHeightInWorld)
   {
      tempFramePoint.setIncludingFrame(pointToProject, 0.0);
      tempFramePoint.changeFrame(worldFrame);

      tempFramePoint.setZ(surfaceHeightInWorld);
      tempFramePoint.changeFrame(centerOfMassFrame);

      double surfaceHeightInCoMFrame = tempFramePoint.getZ();
      tempFramePoint.setZ(controlPlaneHeight.getDoubleValue());

      projectPointThroughReferenceFrame(centerOfMassFrame, tempFramePoint, projectionToPack, surfaceHeightInCoMFrame);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   /**
    * Projects a point that is in the control plane to a point in the world that is on the planar region.
    */
   public void projectPointFromControlPlaneOntoPlanarRegion(ReferenceFrame desiredReferenceFrame,
                                                            FramePoint2DReadOnly pointToProject,
                                                            FramePoint3D projectionToPack,
                                                            PlanarRegion planarRegion)
   {
      tempFramePoint.setIncludingFrame(pointToProject, 0.0);
      tempFramePoint.changeFrame(centerOfMassFrame);
      tempFramePoint.setZ(controlPlaneHeight.getDoubleValue());
      tempFramePoint.changeFrame(worldFrame);

      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      rayDirection.set(tempFramePoint);
      rayDirection.sub(centerOfMassPosition);

      projectionToPack.setToZero(worldFrame);

      planarRegion.getOrigin(planeOrigin);
      planarRegion.getNormal(planeNormal);

      EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(planeOrigin, planeNormal, centerOfMassPosition, rayDirection, projectionToPack);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   public void projectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2DBasics convexPolygonInControlPlaneToPack)
   {
      ConvexPolygon2DReadOnly convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      convexHullInWorld.clear();
      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(convexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectPointsInWorldOntoControlPlane(convexHullInWorld, pointsInPlane);

      convexPolygonInControlPlaneToPack.clear();
      for (int i = 0; i < pointsInPlane.size(); i++)
         convexPolygonInControlPlaneToPack.addVertex(pointsInPlane.get(i));
      convexPolygonInControlPlaneToPack.update();
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion,
                                                                     ConvexPolygon2DBasics convexPolygonInControlPlaneToPack,
                                                                     double distanceInside)
   {
      ConvexPolygon2DReadOnly convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      scaler.scaleConvexPolygon(convexHull, distanceInside, scaledConvexHull);

      for (int i = 0; i < scaledConvexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(scaledConvexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectPointsInWorldOntoControlPlane(convexHullInWorld, pointsInPlane);

      convexPolygonInControlPlaneToPack.clear();
      for (int i = 0; i < pointsInPlane.size(); i++)
         convexPolygonInControlPlaneToPack.addVertex(pointsInPlane.get(i));
      convexPolygonInControlPlaneToPack.update();
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion,
                                                                     ConvexPolygon2DReadOnly footstepPolygon,
                                                                     ConvexPolygon2DBasics convexPolygonInControlPlaneToPack,
                                                                     double distanceInside)
   {
      ConvexPolygon2DReadOnly convexHull = planarRegion.getConvexHull();

      scaler.scaleConvexPolygonToContainInteriorPolygon(convexHull, footstepPolygon, distanceInside, scaledConvexHull);

      for (int i = 0; i < scaledConvexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(scaledConvexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectPointsInWorldOntoControlPlane(convexHullInWorld, pointsInPlane);

      convexPolygonInControlPlaneToPack.clear();
      for (int i = 0; i < pointsInPlane.size(); i++)
         convexPolygonInControlPlaneToPack.addVertex(pointsInPlane.get(i));
      convexPolygonInControlPlaneToPack.update();
   }

   private void projectPointsInWorldOntoControlPlane(List<? extends Point3DReadOnly> convexHullInWorld, RecyclingArrayList<? extends Point2DBasics> pointsInControlPlane)
   {
      pointsInControlPlane.clear();

      for (int vertexIndex = 0; vertexIndex < convexHullInWorld.size(); vertexIndex++)
      {
         tempFramePoint.setIncludingFrame(worldFrame, convexHullInWorld.get(vertexIndex));

         projectPointThroughReferenceFrame(centerOfMassFrame, tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue());

         tempProjectedFramePoint.changeFrame(worldFrame);

         pointsInControlPlane.add().set(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
   }

   private static void projectPointThroughReferenceFrame(ReferenceFrame frameOfProjection,
                                                         FramePoint3DReadOnly pointToProject,
                                                         FramePoint3DBasics projectionToPack,
                                                         double projectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.changeFrame(frameOfProjection);

      double unprojectedHeight = projectionToPack.getZ();
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }
}
