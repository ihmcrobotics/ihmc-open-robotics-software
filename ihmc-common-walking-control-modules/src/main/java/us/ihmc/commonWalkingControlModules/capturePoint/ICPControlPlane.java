package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempProjectedFramePoint = new FramePoint3D();

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

      double heightOfUnprojectedPoint = tempFramePoint.getZ();
      projectPointThroughReferenceFrame(centerOfMassFrame, tempFramePoint, projectionToPack, controlPlaneHeight.getDoubleValue(), heightOfUnprojectedPoint);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

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
      projectPointThroughReferenceFrame(centerOfMassFrame, tempFramePoint, projectionToPack, surfaceHeightInCoMFrame, controlPlaneHeight.getDoubleValue());

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

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

      BoundingBox3D boundingBoxInWorld = planarRegion.getBoundingBox3dInWorld();
      boundingBoxInWorld.intersectionWithRay3D(centerOfMassPosition, rayDirection, projectionToPack, intersectionToThrowAway);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   public void projectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2D convexPolygonInControlPlaneToPack)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      convexHullInWorld.clear();
      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(convexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectConvexPolygonOntoControlPlane(convexHullInWorld, convexPolygonInControlPlaneToPack);
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion,
                                                                     ConvexPolygon2D convexPolygonInControlPlaneToPack,
                                                                     double distanceInside)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      scaler.scaleConvexPolygon(convexHull, distanceInside, scaledConvexHull);

      for (int i = 0; i < scaledConvexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(scaledConvexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectConvexPolygonOntoControlPlane(convexHullInWorld, convexPolygonInControlPlaneToPack);
   }

   public void scaleAndProjectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion,
                                                                     ConvexPolygon2D footstepPolygon,
                                                                     ConvexPolygon2D convexPolygonInControlPlaneToPack,
                                                                     double distanceInside)
   {
      ConvexPolygon2D convexHull = planarRegion.getConvexHull();
      convexPolygonInControlPlaneToPack.clear();

      scaler.scaleConvexPolygonToContainInteriorPolygon(convexHull, footstepPolygon, distanceInside, scaledConvexHull);

      for (int i = 0; i < scaledConvexHull.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = convexHullInWorld.add();
         vertexInWorld.set(scaledConvexHull.getVertex(i));
         planarRegion.getTransformToWorld().transform(vertexInWorld);
      }

      projectConvexPolygonOntoControlPlane(convexHullInWorld, convexPolygonInControlPlaneToPack);
   }

   public void projectConvexPolygonOntoControlPlane(List<? extends Point3DReadOnly> convexHullInWorld, ConvexPolygon2DBasics convexPolygonInControlPlaneToPack)
   {
      convexPolygonInControlPlaneToPack.clear();

      for (int vertexIndex = 0; vertexIndex < convexHullInWorld.size(); vertexIndex++)
      {
         tempFramePoint.setIncludingFrame(worldFrame, convexHullInWorld.get(vertexIndex));
         tempFramePoint.changeFrame(centerOfMassFrame);

         projectPointThroughReferenceFrame(centerOfMassFrame, tempFramePoint, tempProjectedFramePoint, controlPlaneHeight.getDoubleValue(), tempFramePoint.getZ());

         tempProjectedFramePoint.changeFrame(worldFrame);

         convexPolygonInControlPlaneToPack.addVertex(tempProjectedFramePoint.getX(), tempProjectedFramePoint.getY());
      }
      convexPolygonInControlPlaneToPack.update();
   }

   private static void projectPointThroughReferenceFrame(ReferenceFrame frameOfProjection,
                                                         FramePoint3DReadOnly pointToProject,
                                                         FramePoint3DBasics projectionToPack,
                                                         double projectedHeight,
                                                         double unprojectedHeight)
   {
      projectionToPack.setIncludingFrame(pointToProject);
      projectionToPack.changeFrame(frameOfProjection);
      projectionToPack.scale(projectedHeight / unprojectedHeight);
      projectionToPack.setZ(projectedHeight);
   }
}
