package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.function.Function;

public class DiscreteIsoscelesTriangularPrismRayIntersection
{
   public static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
   private final FramePlane3d basePlane = new FramePlane3d();
   private final FramePlane3d leftPlane = new FramePlane3d();
   private final FramePlane3d rightPlane = new FramePlane3d();
   private final FramePlane3d topPlane = new FramePlane3d();
   private final FramePlane3d bottomPlane = new FramePlane3d();
   private final Function<Point3DReadOnly, Boolean> isPointInside = this::isPointInside;
   private final YawPitchRoll tempOrientation = new YawPitchRoll();
   private final FramePoint3D boundingSphereOffset = new FramePoint3D();
   private final RigidBodyTransform shapeTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame shapeFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent("shapeFrame",
                                                                                                                    WORLD_FRAME,
                                                                                                                    shapeTransformToWorld);

   private final Point3D closestIntersection = new Point3D();
   private final Point3D candidatePlaneIntersection = new Point3D();

   public void setup(double triangleWidth,
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

      leftPlane.getPoint().subX(triangleWidth / 2.0);
      tempOrientation.set(0.0, Math.atan2(triangleHeight, triangleWidth / 2.0) + Math.PI / 2.0, 0.0);
      tempOrientation.transform(leftPlane.getNormal());

      rightPlane.getPoint().addX(triangleWidth / 2.0);
      tempOrientation.set(0.0, -Math.atan2(triangleHeight, triangleWidth / 2.0) - Math.PI / 2.0, 0.0);
      tempOrientation.transform(rightPlane.getNormal());

      topPlane.getPoint().addY(prismThickness / 2.0);
      tempOrientation.set(0.0, 0.0, Math.PI / 2.0);
      tempOrientation.transform(topPlane.getNormal());

      bottomPlane.getPoint().subY(prismThickness / 2.0);
      tempOrientation.set(0.0, 0.0, -Math.PI / 2.0);
      tempOrientation.transform(bottomPlane.getNormal());

//      basePlane.getPoint().add(offset);
//      leftPlane.getPoint().add(offset);
//      rightPlane.getPoint().add(offset);
//      topPlane.getPoint().add(offset);
//      bottomPlane.getPoint().add(offset);
//
//      orientation.transform(basePlane.getNormal());
//      orientation.transform(leftPlane.getNormal());
//      orientation.transform(rightPlane.getNormal());
//      orientation.transform(topPlane.getNormal());
//      orientation.transform(bottomPlane.getNormal());

      basePlane.changeFrame(WORLD_FRAME);
      leftPlane.changeFrame(WORLD_FRAME);
      rightPlane.changeFrame(WORLD_FRAME);
      topPlane.changeFrame(WORLD_FRAME);
      bottomPlane.changeFrame(WORLD_FRAME);

      boundingSphereOffset.setIncludingFrame(shapeFrame, 0.0, 0.0, triangleHeight / 2.0);
      boundingSphereOffset.changeFrame(WORLD_FRAME);

      double halfHeight = triangleHeight / 2.0;
      double halfThickness = prismThickness / 2.0;
      double halfWidth = triangleWidth / 2.0;
      double centerToPeakVertex = Math.sqrt(halfHeight * halfHeight + halfThickness * halfThickness);
      double centerToBaseEdge = Math.sqrt(halfHeight * halfHeight + halfWidth * halfWidth);
      double centerToBaseVertex = Math.sqrt(centerToBaseEdge * centerToBaseEdge + halfThickness * halfThickness);

      double boundingRadius = Math.max(centerToBaseVertex, centerToPeakVertex);
//      boundingRadius = 1.0;

      stepCheckIsPointInsideAlgorithm.setup(boundingRadius, boundingSphereOffset);
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
      boolean onOrAboveBasePlane = basePlane.isOnOrAbove(pointToCheck);
      boolean onOrAboveLeftPlane = leftPlane.isOnOrAbove(pointToCheck);
      boolean onOrAboveRightPlane = rightPlane.isOnOrAbove(pointToCheck);
      boolean onOrAboveTopPlane = topPlane.isOnOrAbove(pointToCheck);
      boolean onOrAboveBottomPlane = bottomPlane.isOnOrAbove(pointToCheck);
      return onOrAboveBasePlane && onOrAboveLeftPlane && onOrAboveRightPlane && onOrAboveTopPlane && onOrAboveBottomPlane;
   }

   public Point3D getClosestIntersection()
   {
      return closestIntersection;
   }
}
