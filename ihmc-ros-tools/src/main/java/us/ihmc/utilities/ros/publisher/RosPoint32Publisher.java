package us.ihmc.utilities.ros.publisher;

import geometry_msgs.Point32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

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

   public void publish(Point3DBasics point3f)
   {
      publish(point3f.getX32(), point3f.getY32(), point3f.getZ32());
   }
}
