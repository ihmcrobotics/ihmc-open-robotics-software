package us.ihmc.darpaRoboticsChallenge.ros.messages;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.nio.DoubleBuffer;

public class PoseMessage
{
   /**
    * General Pose message wrapper, as defined in the geometry_msgs/Pose rosmsg type.
    *
    * Structure:
    *
    *    Position (geometry_msgs/Point)
    *       - x
    *       - y
    *       - z
    *    Orientation (geometry_msgs/Quaternion)
    *       - w
    *       - x
    *       - y
    *       - z
    *
    * Note that, internally in ROS, Quaternions are stored x,y,z,w.  We store them w,x,y,z. Translation happens at the native layer.
    */

   private final double[] position = new double[3];
   private final double[] orientation = new double[4];

   public void packPosition(Vector3d position)
   {
      position.setX(this.position[0]);
      position.setY(this.position[1]);
      position.setZ(this.position[2]);
   }

   public void packOrientation(Quat4d orientation)
   {
      orientation.setW((this.orientation[0]));
      orientation.setW((this.orientation[1]));
      orientation.setW((this.orientation[2]));
      orientation.setW((this.orientation[3]));
   }

   public void setFromBuffer(DoubleBuffer buffer)
   {
      buffer.rewind();
      buffer.get(this.position);
      buffer.get(this.orientation);
   }

   public void copyToBuffer(DoubleBuffer buffer)
   {
      buffer.rewind();
      buffer.put(position);
      buffer.put(orientation);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("[PoseMessage] PosX: " + position[0]);
      builder.append("[PoseMessage] PosY: " + position[1]);
      builder.append("[PoseMessage] PosZ: " + position[2]);
      builder.append("[PoseMessage] QuatW: " + orientation[0]);
      builder.append("[PoseMessage] QuatX: " + orientation[1]);
      builder.append("[PoseMessage] QuatY: " + orientation[2]);
      builder.append("[PoseMessage] QuatZ: " + orientation[3]);

      return builder.toString();
   }
}
