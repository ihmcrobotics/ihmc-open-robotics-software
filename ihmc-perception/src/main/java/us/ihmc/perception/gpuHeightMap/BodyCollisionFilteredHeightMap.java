package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.util.List;

public class BodyCollisionFilteredHeightMap
{
   private List<Collidable> robotCollidables;
   private ReferenceFrame cameraFrame;
   private CameraIntrinsics cameraIntrinsics;

   public BodyCollisionFilteredHeightMap()
   {
      this.robotCollidables = null;
      this.cameraFrame = null;
      this.cameraIntrinsics = null;
   }

   public void update(List<Collidable> robotCollidables, ReferenceFrame cameraFrame, CameraIntrinsics cameraIntrinsics)
   {
      if (cameraIntrinsics == null || cameraFrame == null || robotCollidables == null)
      {
         System.err.println("Warning: One or more input parameters are null. Cannot perform update.");
         return;
      }

      this.robotCollidables = robotCollidables;
      this.cameraFrame = cameraFrame;
      this.cameraIntrinsics = cameraIntrinsics;
      double fx = cameraIntrinsics.getFx();
      double fy = cameraIntrinsics.getFy();
      double cx = cameraIntrinsics.getCx();
      double cy = cameraIntrinsics.getCy();
      double width = cameraIntrinsics.getWidth();
      double height = cameraIntrinsics.getHeight();
      System.out.println("Camera Frame Transform to World:\n" + cameraFrame.getTransformToWorldFrame().getTranslation());

      Collidable collidable =  robotCollidables.get(2);

//      for (Collidable collidable : robotCollidables)
//      {
         if (collidable.getShape() instanceof FrameCapsule3D)
         {
            FrameCapsule3D bodypart =  new FrameCapsule3D();
            bodypart.changeFrame(collidable.getShape().getReferenceFrame());
            bodypart.set((FrameCapsule3D) collidable.getShape());
            System.out.println("Original Body Part: " + bodypart);
            bodypart.changeFrame(cameraFrame);
            System.out.println("Body Part in Camera Frame: " + bodypart);

            FramePoint3DReadOnly[] pointsToCheck = {bodypart.getTopCenter(),
                                                    bodypart.getCentroid(),
                                                    bodypart.getBottomCenter()};

            System.out.println("Checking collidable: " + collidable.toString());

            for (FramePoint3DReadOnly point : pointsToCheck)
            {
               if (point.getX() > 0) // Assuming X-axis is forward in the camera frame
               {
                  System.out.println("Point: " + point + " is in front of the camera (X > 0), maybe in view.");
                  double u = fx * (point.getY() / point.getX()) + cx;
                  double v = fy * (point.getZ() / point.getX()) + cy;

                  System.out.println("Projected u: " + u);
                  System.out.println("Projected v: " + v);

                  if (u >= 0 && u < width && v >= 0 && v < height)
                  {
                     System.out.println("In field of view");
                  }
                  else
                  {
                     System.out.println("Out of field of view");
                  }
               }
               else
               {
                  System.out.println("Point: " + point + " is behind or at the camera (X <= 0), not in view.");
               }
            }
            System.out.println("---");
         }
      }
   }
//}
            ////      public static boolean isPointInSphere (Point3D center,double radius, Point3D point)
            ////      {
            ////         double distance = Math.sqrt(
            ////               Math.pow(point.getX() - center.getX(), 2) + Math.pow(point.getY() - center.getY(), 2) + Math.pow(
            ////                     point.getZ() - center.getZ(), 2));
            ////
            ////         if (distance <= radius)
            ////         {
            ////            return true;
            ////         }
            ////         else
            ////         {
            ////            return false;
            ////         }
            ////      }
            ////
            ////      public static boolean isPointInFrameCapsule3D (Point3D topCenter, Point3D bottomCenter, Vector3D axis,
            ////      double radius, Point3D point)
            ////      {
            ////         // Step 1: Check if the point is inside either of the spherical caps (top or bottom)
            ////         if (isPointInSphere(topCenter, radius, point) || isPointInSphere(bottomCenter, radius, point))
            ////         {
            ////            return true; // Point is inside one of the spherical caps
            ////         }
            ////
            ////         // Step 2: Check if the point is inside the cylindrical portion
            ////         // Calculate the vector from bottomCenter to point
            ////         Vector3D pointToBottom = new Vector3D(point.getX() - bottomCenter.getX(),
            ////                                               point.getY() - bottomCenter.getY(),
            ////                                               point.getZ() - bottomCenter.getZ());
            ////
            ////         // Project pointToBottom onto the axis (direction vector) to get the distance along the capsule's axis
            ////         double projection = pointToBottom.dot(axis);
            ////
            ////         // Check if the projection lies within the bounds of the cylindrical section
            ////         double capsuleLength = Math.sqrt(
            ////               Math.pow(topCenter.getX() - bottomCenter.getX(), 2) + Math.pow(topCenter.getY() - bottomCenter.getY(), 2)
            ////               + Math.pow(topCenter.getZ() - bottomCenter.getZ(), 2));
            ////         if (projection >= 0 && projection <= capsuleLength)
            ////         {
            ////            // Check if the perpendicular distance from the point to the axis is less than or equal to the radius
            ////            double distanceToAxis = getDistanceFromLine(axis, bottomCenter, point);
            ////            return distanceToAxis <= radius;
            ////         }
            ////
            ////         // If the point is outside the cylindrical portion and spherical caps, return false
            ////         return false;
            ////      }
            ////
            ////      // Helper method to calculate the perpendicular distance from a point to a line (capsule axis)
            ////      public static double getDistanceFromLine (Vector3D axisDirection, Point3D linePoint, Point3D point)
            ////      {
            ////         // Vector from linePoint to the point
            ////         Vector3D pointToLine = new Vector3D(point.getX() - linePoint.getX(),
            ////                                             point.getY() - linePoint.getY(),
            ////                                             point.getZ() - linePoint.getZ());
            ////
            ////         // Project pointToLine onto the axis direction and subtract to get the perpendicular vector
            ////         double projection = pointToLine.dot(axisDirection);
            ////
            ////         // Calculate the vector component along the axis
            ////         Vector3D projectedVector = new Vector3D(axisDirection);
            ////         projectedVector.scale(projection);
            ////
            ////         // Calculate the perpendicular vector
            ////         Vector3D perpendicular = new Vector3D(pointToLine.getX() - projectedVector.getX(),
            ////                                               pointToLine.getY() - projectedVector.getY(),
            ////                                               pointToLine.getZ() - projectedVector.getZ());
            ////
            ////         // Return the length of the perpendicular vector (distance to the line)
            ////         return perpendicular.length();
            ////      }
            //   }
