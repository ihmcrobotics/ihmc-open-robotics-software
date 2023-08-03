package us.ihmc.avatar.networkProcessor.lidarScanPublisher;

import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ShadowScanPointFilter implements ScanPointFilter
{
   public static final double DEFAULT_SHADOW_ANGLE_THRESHOLD = Math.toRadians(12.0);

   private double shadowAngleThreshold = DEFAULT_SHADOW_ANGLE_THRESHOLD;

   private final Vector3D fromLidarToScanPoint = new Vector3D();
   private final Vector3D currentToNextScanPoint = new Vector3D();
   private final Vector3D previousToCurrentScanPoint = new Vector3D();

   private Point3DReadOnly sensorPosition;
   private PointCloudData pointCloudData;

   public ShadowScanPointFilter()
   {
   }

   /**
    * <p>
    * For more details, see
    * <a href="http://groups.csail.mit.edu/robotics-center/public_papers/Marion16a.pdf"> Pat Marion
    * master thesis, section 2.2.1, page 25.</a>
    * </p>
    * 
    * @param shadowAngleThreshold the angle threshold in radians used by the removal algorithm.
    *                             Expecting a positive value close to zero, the default value is 0.21
    *                             radian (= 12 degrees).
    */
   public void setShadowAngleThreshold(double shadowAngleThreshold)
   {
      this.shadowAngleThreshold = shadowAngleThreshold;
   }

   public void set(Point3DReadOnly sensorPosition, PointCloudData pointCloudData)
   {
      this.sensorPosition = sensorPosition;
      this.pointCloudData = pointCloudData;
   }

   /**
    * Attempt to remove flying LIDAR points, which, when present, result as objects having shadows.
    * <p>
    * Warning: The algorithm for removing shadows expects to be dealing with single LIDAR scans.
    * </p>
    * <p>
    * The rejection method is based on the observation that flying points always fall in line with view
    * direction of the laser ray. It compares the angle between the angle between the scanner view
    * direction and the line segment connecting outlier points with their scan line neighbors.
    * </p>
    * <p>
    * For more details, see
    * <a href="http://groups.csail.mit.edu/robotics-center/public_papers/Marion16a.pdf"> Pat Marion
    * master thesis, section 2.2.1, page 25.</a>
    * </p>
    */
   @Override
   public boolean test(int index, Point3DReadOnly currentScanPoint)
   {
      if (shadowAngleThreshold <= 0.0 || index == 0 || index == pointCloudData.getNumberOfPoints() - 1)
         return true;

      Point3D[] pointCloud = pointCloudData.getPointCloud();

      // FIXME Need to remove the use of Vector3DReadOnly.angle(...).

      Point3D previousScanPoint = pointCloud[index - 1];
      fromLidarToScanPoint.sub(currentScanPoint, sensorPosition);
      previousToCurrentScanPoint.sub(currentScanPoint, previousScanPoint);

      if (fromLidarToScanPoint.dot(previousToCurrentScanPoint) < 0.0)
         previousToCurrentScanPoint.negate();
      if (fromLidarToScanPoint.angle(previousToCurrentScanPoint) < shadowAngleThreshold)
         return false;

      Point3D nextScanPoint = pointCloud[index + 1];
      currentToNextScanPoint.sub(nextScanPoint, currentScanPoint);

      if (fromLidarToScanPoint.dot(currentToNextScanPoint) < 0.0)
         currentToNextScanPoint.negate();
      if (fromLidarToScanPoint.angle(currentToNextScanPoint) < shadowAngleThreshold)
         return false;

      return true;
   }
}
