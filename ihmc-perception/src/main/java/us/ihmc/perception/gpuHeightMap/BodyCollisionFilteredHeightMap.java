package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class BodyCollisionFilteredHeightMap
{
   //   public static void getRobotBodyShapes(FullHumanoidRobotModel fullRobotModel)
   //   {
   //      for (RobotSide robotSide : RobotSide.values)
   //
   //      {
   //         List<JointBasics> joints = new ArrayList<>();
   //         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide)
   //                                              .getParentJoint()
   //                                              .getPredecessor()
   //                                              .getParentJoint()
   //                                              .getPredecessor();
   //         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
   //         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
   //      }
   //      return new
   //
   //            CollidingScanRegionFilter(shapeTester);
   //   }

   public static boolean isPointInSphere(Point3D center, double radius, Point3D point)
   {
      double distance = Math.sqrt(
            Math.pow(point.getX() - center.getX(), 2) + Math.pow(point.getY() - center.getY(), 2) + Math.pow(
                  point.getZ() - center.getZ(), 2));

      if (distance <= radius)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public static boolean isPointInFrameCapsule3D(Point3D topCenter,
                                                 Point3D bottomCenter,
                                                 Vector3D axis,
                                                 double radius,
                                                 Point3D point)
   {
      // Step 1: Check if the point is inside either of the spherical caps (top or bottom)
      if (isPointInSphere(topCenter, radius, point) || isPointInSphere(bottomCenter, radius, point))
      {
         return true; // Point is inside one of the spherical caps
      }

      // Step 2: Check if the point is inside the cylindrical portion
      // Calculate the vector from bottomCenter to point
      Vector3D pointToBottom = new Vector3D(point.getX() - bottomCenter.getX(),
                                            point.getY() - bottomCenter.getY(),
                                            point.getZ() - bottomCenter.getZ());

      // Project pointToBottom onto the axis (direction vector) to get the distance along the capsule's axis
      double projection = pointToBottom.dot(axis);

      // Check if the projection lies within the bounds of the cylindrical section
      double capsuleLength = Math.sqrt(
            Math.pow(topCenter.getX() - bottomCenter.getX(), 2) + Math.pow(topCenter.getY() - bottomCenter.getY(), 2)
            + Math.pow(topCenter.getZ() - bottomCenter.getZ(), 2));
      if (projection >= 0 && projection <= capsuleLength)
      {
         // Check if the perpendicular distance from the point to the axis is less than or equal to the radius
         double distanceToAxis = getDistanceFromLine(axis, bottomCenter, point);
         return distanceToAxis <= radius;
      }

      // If the point is outside the cylindrical portion and spherical caps, return false
      return false;
   }

   // Helper method to calculate the perpendicular distance from a point to a line (capsule axis)
   public static double getDistanceFromLine(Vector3D axisDirection, Point3D linePoint, Point3D point)
   {
      // Vector from linePoint to the point
      Vector3D pointToLine = new Vector3D(point.getX() - linePoint.getX(),
                                          point.getY() - linePoint.getY(),
                                          point.getZ() - linePoint.getZ());

      // Project pointToLine onto the axis direction and subtract to get the perpendicular vector
      double projection = pointToLine.dot(axisDirection);

      // Calculate the vector component along the axis
      Vector3D projectedVector = new Vector3D(axisDirection);
      projectedVector.scale(projection);

      // Calculate the perpendicular vector
      Vector3D perpendicular = new Vector3D(pointToLine.getX() - projectedVector.getX(),
                                            pointToLine.getY() - projectedVector.getY(),
                                            pointToLine.getZ() - projectedVector.getZ());

      // Return the length of the perpendicular vector (distance to the line)
      return perpendicular.length();
   }
}



