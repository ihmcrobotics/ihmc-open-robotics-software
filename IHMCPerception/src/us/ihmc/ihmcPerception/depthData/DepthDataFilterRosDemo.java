package us.ihmc.ihmcPerception.depthData;

import java.net.URISyntaxException;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.ros.apps.RosPointCloudFilterRepublisher;

/*
 * See @RosPointCloudFilterRepublisher
 */
public class DepthDataFilterRosDemo extends RosPointCloudFilterRepublisher
{
   private DepthDataFilter depthDataFilter;
   private Point3d sensorOrigin = new Point3d(0.0,0.0,1.0);


   public DepthDataFilterRosDemo()
   {
      this.depthDataFilter = new DepthDataFilter();

   }


   @Override
   protected boolean includePoint(Point3d point, Color3f color)
   {
      return includePoint(point, (color.x + color.y + color.z) / 3.0f);
   }

   @Override
   protected boolean includePoint(Point3d point, float intensity)
   {
      return depthDataFilter.addPoint(point, sensorOrigin);
   }


   public static void main(String[] arg) throws URISyntaxException
   {
      DepthDataFilterRosDemo republisher = new DepthDataFilterRosDemo();
      new Thread(republisher).start();
   }
}
