package us.ihmc.atlas.sensors;

import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import scan_to_cloud.PointCloud2WithSource;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

/**
 * Gets a single lidar scan in world and gives the pose of the lidar frame in head. You will need to finish the transformation to world. 
 *
 */
public class PointCloudWithSourcePoseTester extends AbstractRosTopicSubscriber<PointCloud2WithSource>
{

   public PointCloudWithSourcePoseTester(RosMainNode rosMainNode)
   {
      super(PointCloud2WithSource._TYPE);
      rosMainNode.attachSubscriber("/singleScanAsCloudWithSource", this);
   }

   @Override
   public void onNewMessage(PointCloud2WithSource cloudHolder)
   {
      System.out.println(cloudHolder.getCloud().getWidth());
      Point translation = cloudHolder.getTranslation();
      Point3d position = new Point3d(translation.getX(), translation.getY(),translation.getZ());
      System.out.println(position);
      Quaternion orientation = cloudHolder.getOrientation();
      Quat4d quaternion = new Quat4d(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getW());
      System.out.println(quaternion);
   }
   
   public static void main(String[] args) throws URISyntaxException
   {
      URI masterURI = new URI("http://cpu0:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "scanToCLoudJavaTester", true);
      new PointCloudWithSourcePoseTester(rosMainNode);
      rosMainNode.execute();
   }

}
