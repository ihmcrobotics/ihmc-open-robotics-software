package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

public class ContactPlaneProviderTools
{
   public static final double gridSizeXY = 0.05;
   public static final double rateGridSizeXY = 0.1;
   public static final int yawDivisions = 36;
   public static final double gridSizeYaw = 2.0 * Math.PI / yawDivisions;

   public static final double timeGridSize = 0.01;

   public static final double areaGridSize = gridSizeXY * gridSizeXY;

   public static final int prime = 31;

   public static int computePlaneHashCode(Pose3DReadOnly pose, ConvexPolygon2DReadOnly polygon)
   {
      int result = computePointHashCode(pose.getPosition());

      result += prime * Math.floorMod((int) (Math.round((pose.getYaw()) / gridSizeYaw)), yawDivisions);
      result += prime * (int) Math.round(polygon.getArea() / areaGridSize);

      return result;
   }

   public static int computePointHashCode(Point3DReadOnly point)
   {
      int result = 1;

      result += prime * (int) Math.round(point.getX() / gridSizeXY);
      result += prime * (int) Math.round(point.getY() / gridSizeXY);

      return result;
   }

   public static int computeVectorHashCode(Vector3DReadOnly vector)
   {
      int result = 1;

      result += prime * (int) Math.round(vector.getX() / rateGridSizeXY);
      result += prime * (int) Math.round(vector.getY() / rateGridSizeXY);

      return result;
   }

   public static int computePlaneProviderHashCode(int planeHashCode,
                                                  int timeHashCode,
                                                  int startHashCode,
                                                  int startVelHashCode,
                                                  int endHashCode,
                                                  int endVelHashCode)
   {
      int result = 1;

      result += prime * planeHashCode;
      result += prime * timeHashCode;
      result += prime * startHashCode;
      result += prime * startVelHashCode;
      result += prime * endHashCode;
      result += prime * endVelHashCode;

      return result;
   }

   public static int computeTimeHashCode(TimeIntervalReadOnly timeInterval)
   {
      int result = 1;

      result += prime * (int) Math.round(timeInterval.getDuration() / timeGridSize);
      result += prime * (int) Math.round(timeInterval.getEndTime() / timeGridSize);

      return result;
   }
}
