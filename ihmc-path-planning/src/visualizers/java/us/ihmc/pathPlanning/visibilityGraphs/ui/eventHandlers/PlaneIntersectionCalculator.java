package us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers;

import com.sun.javafx.scene.CameraHelper;
import javafx.event.EventHandler;
import javafx.scene.Camera;
import javafx.scene.input.MouseEvent;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Calculates intersection of mouse position on plane of the given PlanarRegion (ignores the polygon)
 */
public class PlaneIntersectionCalculator implements EventHandler<MouseEvent>
{
   private final AtomicReference<PlanarRegion> planarRegion = new AtomicReference<>();
   private final AtomicReference<Point3D> intersectionPoint = new AtomicReference<>();
   private final Camera camera;

   public PlaneIntersectionCalculator(Camera camera)
   {
      this.camera = camera;
   }

   @Override
   public void handle(MouseEvent event)
   {
      if(planarRegion.get() == null)
         return;

      intersectionPoint.set(intersectRayWithPlane(camera, planarRegion.get(), event));
   }

   private static Point3D intersectRayWithPlane(Camera camera, PlanarRegion planarRegion, MouseEvent event)
   {
      Line3D line = getPickRay(camera, event);

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionTransform);
      Vector3DReadOnly planeNormal = planarRegion.getNormal();
      Point3DReadOnly pointOnPlane = planarRegion.getPoint();

      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, line.getPoint(), line.getDirection());
   }

   private static Line3D getPickRay(Camera camera, MouseEvent event)
   {
      Point3D point1 = new Point3D();
      point1.setX(camera.getLocalToSceneTransform().getTx());
      point1.setY(camera.getLocalToSceneTransform().getTy());
      point1.setZ(camera.getLocalToSceneTransform().getTz());

      Point3D point2 = new Point3D();
      javafx.geometry.Point3D pointOnProjectionPlane = CameraHelper.pickProjectPlane(camera, event.getSceneX(), event.getSceneY());
      point2.setX(pointOnProjectionPlane.getX());
      point2.setY(pointOnProjectionPlane.getY());
      point2.setZ(pointOnProjectionPlane.getZ());

      return new Line3D(point1, point2);
   }

   public Point3D pollIntersection()
   {
      return intersectionPoint.getAndSet(null);
   }

   public void setPlanarRegion(PlanarRegion planarRegion)
   {
      this.planarRegion.set(planarRegion);
   }
}
