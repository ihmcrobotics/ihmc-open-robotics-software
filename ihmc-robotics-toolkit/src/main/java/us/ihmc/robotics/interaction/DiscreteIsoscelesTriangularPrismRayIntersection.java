package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;

public class DiscreteIsoscelesTriangularPrismRayIntersection
{
   public static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private final FramePlane3d basePlane = new FramePlane3d();
   private final FramePlane3d leftPlane = new FramePlane3d();
   private final FramePlane3d rightPlane = new FramePlane3d();
   private final FramePlane3d topPlane = new FramePlane3d();
   private final FramePlane3d bottomPlane = new FramePlane3d();
   private final YawPitchRoll tempOrientation = new YawPitchRoll();
   private final RigidBodyTransform shapeTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame shapeFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("shapeFrame",
                                                                                                             WORLD_FRAME,
                                                                                                             shapeTransformToWorld);

   private final Point3D closestIntersection = new Point3D();
   private final Point3D candidatePlaneIntersection = new Point3D();

   public void update(double triangleWidth,
                      double triangleHeight,
                      double prismThickness,
                      Tuple3DReadOnly offset,
                      Orientation3DReadOnly orientation,
                      RigidBodyTransformReadOnly transform)
   {
      shapeTransformToWorld.set(transform);
      shapeTransformToWorld.appendTranslation(offset);
      shapeTransformToWorld.appendOrientation(orientation);
      shapeFrame.update();

      basePlane.setToZero(shapeFrame);
      leftPlane.setToZero(shapeFrame);
      rightPlane.setToZero(shapeFrame);
      topPlane.setToZero(shapeFrame);
      bottomPlane.setToZero(shapeFrame);

      double lowerInnerAngle = Math.PI / 2.0 - Math.atan2(triangleHeight, triangleWidth / 2.0);
      leftPlane.getPoint().subX(triangleWidth / 2.0);
      tempOrientation.set(0.0, lowerInnerAngle + Math.PI / 2.0, 0.0);
      tempOrientation.transform(leftPlane.getNormal());

      rightPlane.getPoint().addX(triangleWidth / 2.0);
      tempOrientation.set(0.0, -lowerInnerAngle - Math.PI / 2.0, 0.0);
      tempOrientation.transform(rightPlane.getNormal());

      topPlane.getPoint().addY(prismThickness / 2.0);
      tempOrientation.set(0.0, 0.0, Math.PI / 2.0);
      tempOrientation.transform(topPlane.getNormal());

      bottomPlane.getPoint().subY(prismThickness / 2.0);
      tempOrientation.set(0.0, 0.0, -Math.PI / 2.0);
      tempOrientation.transform(bottomPlane.getNormal());

      basePlane.changeFrame(WORLD_FRAME);
      leftPlane.changeFrame(WORLD_FRAME);
      rightPlane.changeFrame(WORLD_FRAME);
      topPlane.changeFrame(WORLD_FRAME);
      bottomPlane.changeFrame(WORLD_FRAME);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      closestIntersection.setToNaN();
      double closestDistance = Double.POSITIVE_INFINITY;

      basePlane.getIntersectionWithLine(candidatePlaneIntersection, pickRay);
      if (isPointInside(candidatePlaneIntersection))
      {
         double distance = candidatePlaneIntersection.distance(pickRay.getPoint());
         if (distance < closestDistance)
         {
            closestIntersection.set(candidatePlaneIntersection);
            closestDistance = distance;
         }
      }
      leftPlane.getIntersectionWithLine(candidatePlaneIntersection, pickRay);
      if (isPointInside(candidatePlaneIntersection))
      {
         double distance = candidatePlaneIntersection.distance(pickRay.getPoint());
         if (distance < closestDistance)
         {
            closestIntersection.set(candidatePlaneIntersection);
            closestDistance = distance;
         }
      }
      rightPlane.getIntersectionWithLine(candidatePlaneIntersection, pickRay);
      if (isPointInside(candidatePlaneIntersection))
      {
         double distance = candidatePlaneIntersection.distance(pickRay.getPoint());
         if (distance < closestDistance)
         {
            closestIntersection.set(candidatePlaneIntersection);
            closestDistance = distance;
         }
      }
      bottomPlane.getIntersectionWithLine(candidatePlaneIntersection, pickRay);
      if (isPointInside(candidatePlaneIntersection))
      {
         double distance = candidatePlaneIntersection.distance(pickRay.getPoint());
         if (distance < closestDistance)
         {
            closestIntersection.set(candidatePlaneIntersection);
            closestDistance = distance;
         }
      }
      topPlane.getIntersectionWithLine(candidatePlaneIntersection, pickRay);
      if (isPointInside(candidatePlaneIntersection))
      {
         double distance = candidatePlaneIntersection.distance(pickRay.getPoint());
         if (distance < closestDistance)
         {
            closestIntersection.set(candidatePlaneIntersection);
            closestDistance = distance;
         }
      }

      return closestIntersection.containsNaN() ? Double.NaN : closestDistance;
   }

   private boolean isPointInside(Point3DReadOnly pointToCheck)
   {
      boolean onOrAboveBasePlane = basePlane.isOnOrAbove(pointToCheck, 1e-7);
      boolean onOrAboveLeftPlane = leftPlane.isOnOrAbove(pointToCheck, 1e-7);
      boolean onOrAboveRightPlane = rightPlane.isOnOrAbove(pointToCheck, 1e-7);
      boolean onOrAboveTopPlane = topPlane.isOnOrAbove(pointToCheck, 1e-7);
      boolean onOrAboveBottomPlane = bottomPlane.isOnOrAbove(pointToCheck, 1e-7);
      return onOrAboveBasePlane && onOrAboveLeftPlane && onOrAboveRightPlane && onOrAboveTopPlane && onOrAboveBottomPlane;
   }

   public Point3D getClosestIntersection()
   {
      return closestIntersection;
   }
}
