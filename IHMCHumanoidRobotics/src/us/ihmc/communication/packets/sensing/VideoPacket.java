package us.ihmc.communication.packets.sensing;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.random.RandomTools;
import boofcv.struct.calib.IntrinsicParameters;

@HighBandwidthPacket
public class VideoPacket extends Packet<VideoPacket> implements TransformableDataObject<VideoPacket>
{
   public RobotSide robotSide;
   public long timeStamp;
   public byte[] data;
   public Point3d position;
   public Quat4d orientation;
   public IntrinsicParameters intrinsicParameters;
   
   public VideoPacket()
   {
      setDestination(PacketDestination.UI);
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public byte[] getData()
   {
      return data;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }


   public VideoPacket(RobotSide robotSide, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
   {
      setDestination(PacketDestination.UI);
      this.robotSide = robotSide;
      this.timeStamp = timeStamp;
      this.data = data;
      this.position = position;
      this.orientation = orientation;
      this.intrinsicParameters = intrinsicParameters;
   }

   public boolean epsilonEquals(VideoPacket other, double epsilon)
   {
      if (!getPosition().epsilonEquals(other.getPosition(), epsilon))
      {
         System.out.println(getPosition());
         System.out.println(other.getPosition());

         return false;
      }

      if (!RotationFunctions.quaternionEpsilonEquals(getOrientation(), other.getOrientation(), epsilon))
      {
         System.out.println(getOrientation());
         System.out.println(other.getOrientation());

         return false;
      }


      if (this.getData().length != other.getData().length)
      {
         System.out.println("Data length");
         System.out.println(getData().length);
         System.out.println(other.getData().length);

         return false;
      }

      for (int i = 0; i < getData().length; i++)
      {
         if (getData()[i] != other.getData()[i])
         {
            return false;
         }
      }

      return true;
   }

   public VideoPacket transform(RigidBodyTransform transform)
   {
      Point3d newPoint = TransformTools.getTransformedPoint(getPosition(), transform);
      Quat4d newOrientation = TransformTools.getTransformedQuat(getOrientation(), transform);

      return new VideoPacket(getRobotSide(), getTimeStamp(), getData(), newPoint, newOrientation, getIntrinsicParameters());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   
   
   public IntrinsicParameters getIntrinsicParameters()
   {
      return intrinsicParameters;
   }

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {
      this.intrinsicParameters = intrinsicParameters;
   }

   @Override
   public String toString()
   {
      return "VideoPacket [timeStamp=" + timeStamp + ", data=" + data.length + " byte, position=" + position + ", orientation=" + orientation;
   }

   public VideoPacket(Random random)
   {
      int[] POSITION_BITS = { 16, 16, 16 };
      int QUATERNION_BITS = 14;
      int SOURCE_ID_BITS = 8;

      int positionOffset = 32;
      int rotationOffset = positionOffset + POSITION_BITS[0] + POSITION_BITS[1] + POSITION_BITS[2];
      int metaDataLength = rotationOffset + 4 * QUATERNION_BITS + SOURCE_ID_BITS;

      int length = random.nextInt((int) (Math.pow(2, 20) - 1 - metaDataLength / 8));

      //    int length = (int) (Math.pow(2, 20) - 1 - metaDataLength/8);
      byte[] data = new byte[length];
      random.nextBytes(data);

      Point3d position = RandomTools.generateRandomPoint(random, 2.0, 2.0, 1.0);
      Quat4d orientation = RandomTools.generateRandomQuaternion(random);

      this.timeStamp = 0;
      this.data = data;
      this.position = position;
      this.orientation = orientation;
   }
}
