package us.ihmc.avatar.ros.messages;

import java.nio.DoubleBuffer;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

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

   public void packPosition(Vector3D position)
   {
      position.setX(this.position[0]);
      position.setY(this.position[1]);
      position.setZ(this.position[2]);
   }

   public void packOrientation(Quaternion orientation)
   {
      // FIXME used to be that before switching to EuclidCore
//      orientation.setW((this.orientation[0]));
//      orientation.setW((this.orientation[1]));
//      orientation.setW((this.orientation[2]));
//      orientation.setW((this.orientation[3]));
      orientation.set(this.orientation);
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

   public void setPosition(double[] position)
   {
      if(position.length < 3)
      {
         throw new RuntimeException("Bad position vector");
      }
      this.position[0] = position[0];
      this.position[1] = position[1];
      this.position[2] = position[2];
   }

   public double getPosX()
   {
      return position[0];
   }

   public double getPosY()
   {
      return position[1];
   }
   public double getPosZ()
   {
      return position[2];
   }

   public double getOrientationW()
   {
      return orientation[0];
   }

   public double getOrientationX()
   {
      return orientation[1];
   }

   public double getOrientationY()
   {
      return orientation[2];
   }

   public double getOrientationZ()
   {
      return orientation[3];
   }

   public void setOrientation(double[] orientation)
   {
      if(position.length < 4)
      {
         throw new RuntimeException("Bad position vector");
      }
      this.orientation[0] = orientation[0];
      this.orientation[1] = orientation[1];
      this.orientation[2] = orientation[2];
      this.orientation[3] = orientation[3];
   }

   public void setPosX(double x)
   {
      this.position[0] = x;
   }

   public void setPosY(double y)
   {
      this.position[1] = y;
   }

   public void setPosZ(double z)
   {
      this.position[2] = z;
   }

   public void setOrientationW(double w)
   {
      this.orientation[0] = w;
   }

   public void setOrientationX(double x)
   {
      this.orientation[1] = x;
   }

   public void setOrientationY(double y)
   {
      this.orientation[2] = y;
   }

   public void setOrientationZ(double z)
   {
      this.orientation[3] = z;
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
