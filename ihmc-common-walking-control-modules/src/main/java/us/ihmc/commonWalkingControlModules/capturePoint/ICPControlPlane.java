package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ICPControlPlane
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble controlPlaneHeight;
   private final ReferenceFrame centerOfMassFrame;

   private final RecyclingArrayList<Point3D> vertexInWorldProvider = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point2DBasics> pointsInPlane = new RecyclingArrayList<>(Point2D::new);

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempProjectedFramePoint = new FramePoint3D();

   private final FramePoint3D planeOrigin = new FramePoint3D();
   private final FrameVector3D planeNormal = new FrameVector3D();

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();

   private final double gravityZ;

   public ICPControlPlane(ReferenceFrame centerOfMassFrame, double gravityZ, YoRegistry parentRegistry)
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
      planarRegion.getOrigin(planeOrigin);
      planarRegion.getNormal(planeNormal);

      projectPointFromControlPlaneOntoRegion(desiredReferenceFrame, pointToProject, projectionToPack, planeOrigin, planeNormal);
   }

   public void projectPointFromControlPlaneOntoConstraintRegion(ReferenceFrame desiredReferenceFrame,
                                                                FramePoint2DReadOnly pointToProject,
                                                                FramePoint3D projectionToPack,
                                                                StepConstraintRegion stepConstraintRegion)
   {
      stepConstraintRegion.getNormal(planeNormal);
      stepConstraintRegion.getRegionOriginInWorld(planeOrigin);

      projectPointFromControlPlaneOntoRegion(desiredReferenceFrame, pointToProject, projectionToPack, planeOrigin, planeNormal);
   }

   public void projectPointFromControlPlaneOntoRegion(ReferenceFrame desiredReferenceFrame,
                                                                FramePoint2DReadOnly pointToProject,
                                                                FramePoint3D projectionToPack,
                                                                Point3DReadOnly regionOrigin,
                                                                FrameVector3DReadOnly regionNormal)
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

      EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(regionOrigin, regionNormal, centerOfMassPosition, rayDirection, projectionToPack);

      projectionToPack.changeFrame(desiredReferenceFrame);
   }

   public void projectPlanarRegionConvexHullOntoControlPlane(PlanarRegion planarRegion, ConvexPolygon2DBasics convexPolygonInControlPlaneToPack)
   {
      projectVerticesOntoControlPlane(planarRegion.getConvexHull(), planarRegion.getTransformToWorld(), convexPolygonInControlPlaneToPack);
   }

   public void projectPlanarRegionConvexHullInWorldOntoControlPlane(ConvexPolygon2DReadOnly convexHullInWorld, PlanarRegion heightProvider,
                                                                    ConvexPolygon2DBasics convexPolygonInControlPlaneToPack)
   {
      vertexInWorldProvider.clear();
      for (int i = 0; i < convexHullInWorld.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = vertexInWorldProvider.add();
         Point2DReadOnly vertex = convexHullInWorld.getVertex(i);
         vertexInWorld.set(vertex, heightProvider.getPlaneZGivenXY(vertex.getX(), vertex.getY()));
      }

      projectPointsInWorldOntoControlPlane(vertexInWorldProvider, pointsInPlane);

      convexPolygonInControlPlaneToPack.clear();
      for (int i = 0; i < pointsInPlane.size(); i++)
         convexPolygonInControlPlaneToPack.addVertex(pointsInPlane.get(i));
      convexPolygonInControlPlaneToPack.update();
   }

   public void projectVerticesOntoControlPlane(Vertex2DSupplier convexHullInLocal, RigidBodyTransformReadOnly transformFromLocalToWorld,
                                               ConvexPolygon2DBasics convexPolygonInControlPlaneToPack)
   {
      vertexInWorldProvider.clear();
      for (int i = 0; i < convexHullInLocal.getNumberOfVertices(); i++)
      {
         Point3D vertexInWorld = vertexInWorldProvider.add();
         vertexInWorld.set(convexHullInLocal.getVertex(i), 0.0);
         transformFromLocalToWorld.transform(vertexInWorld);
      }

      projectPointsInWorldOntoControlPlane(vertexInWorldProvider, pointsInPlane);

      convexPolygonInControlPlaneToPack.clear();
      for (int i = 0; i < pointsInPlane.size(); i++)
         convexPolygonInControlPlaneToPack.addVertex(pointsInPlane.get(i));
      convexPolygonInControlPlaneToPack.update();
   }

   public void projectPointsInWorldOntoControlPlane(List<? extends Point3DReadOnly> convexHullInWorld, RecyclingArrayList<? extends Point2DBasics> pointsInControlPlane)
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
