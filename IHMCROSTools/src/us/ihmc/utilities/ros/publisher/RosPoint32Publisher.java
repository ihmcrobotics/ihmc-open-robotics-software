package us.ihmc.utilities.ros.publisher;

import geometry_msgs.Point32;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosPoint32Publisher extends RosTopicPublisher<geometry_msgs.Point32>
{
   public RosPoint32Publisher(boolean latched)
   {
      super(geometry_msgs.Point32._TYPE, latched);
   }

   public void publish(float x, float y, float z)
   {
      Point32 message = getMessage();
      message.setX(x);
      message.setY(y);
      message.setZ(z);
      publish(message);
   }

   public void publish(Point3d point3d)
   {
      publish((float) point3d.getX(), (float) point3d.getY(), (float) point3d.getZ());
   }

   public void publish(Point3f point3f)
   {
      publish(point3f.getX(), point3f.getY(), point3f.getZ());
   }
}
