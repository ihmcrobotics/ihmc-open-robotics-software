package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;

public class SnapFootstepPacket extends Packet<SnapFootstepPacket>
{
   //   public static final byte UNKOWN = 0;
   //   public static final byte VALID_UNCHANGED_STEP = 1;
   //   public static final byte VALID_SNAPPED_STEP = 2;
   //   public static final byte BAD_STEP = 3;
   public ArrayList<FootstepDataMessage> footstepData;
   public int[] footstepOrder;
   public byte[] flag;

   public SnapFootstepPacket()
   {
      // Empty constructor for deserialization
   }

   public SnapFootstepPacket(ArrayList<FootstepDataMessage> data, int[] footstepOrder, byte[] flag)
   {
      this.footstepData = data;
      this.footstepOrder = footstepOrder;
      this.flag = flag;
   }

   public ArrayList<FootstepDataMessage> getFootstepData()
   {
      return this.footstepData;
   }

   public int[] getFootstepOrder()
   {
      return footstepOrder;
   }

   public byte[] getFlag()
   {
      return flag;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof SnapFootstepPacket) && this.epsilonEquals((SnapFootstepPacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(SnapFootstepPacket other, double epsilon)
   {
      boolean ret = true;
      if(this.footstepData.size() != other.footstepData.size())
      {
         return false;
      }

      for (int i = 0; i < footstepData.size(); i++)
      {
         ret &= this.footstepData.get(i).epsilonEquals(other.footstepData.get(i), epsilon);
         ret &= this.footstepOrder[i] == other.footstepOrder[i];
         ret &= this.flag[i] == other.flag[i];
      }

      return ret;
   }

   public SnapFootstepPacket(Random random)
   {
      // Number of footsteps
      int numberOfFootsteps = random.nextInt(255);

      // create random footsteps
      int[] footstepOrder = new int[numberOfFootsteps];
      byte[] flag = new byte[numberOfFootsteps];
      ArrayList<FootstepDataMessage> footsteps = new ArrayList<FootstepDataMessage>();
      RigidBodyTransform previousFootstep = new RigidBodyTransform();

      double[] XYZ_MAX = {2.0, 2.0, 2.0};
      double[] XYZ_MIN = {-2.0, -2.0, -3.0};

      double xMax = 0.90 * Math.min(Math.abs(XYZ_MAX[0]), Math.abs(XYZ_MIN[0]));
      double yMax = 0.90 * Math.min(Math.abs(XYZ_MAX[1]), Math.abs(XYZ_MIN[1]));
      double zMax = 0.90 * Math.min(Math.abs(XYZ_MAX[2]), Math.abs(XYZ_MIN[2]));

      for (int footstepNumber = 0; footstepNumber < numberOfFootsteps; footstepNumber++)
      {
         footstepOrder[footstepNumber] = footstepNumber;
         flag[footstepNumber] = (byte) random.nextInt(3);
         RobotSide robotSide = (footstepNumber % 2 == 0) ? RobotSide.RIGHT : RobotSide.LEFT;

         Point3D position = RandomGeometry.nextPoint3D(random, xMax, yMax, zMax);

         Quaternion orientation = new Quaternion();
         orientation.set(RandomGeometry.nextAxisAngle(random));

         previousFootstep.transform(position);

         previousFootstep.setTranslation(new Vector3D32(position));
         previousFootstep.setRotation(orientation);

         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, new Point3D(position), orientation);

         footsteps.add(footstepData);
      }

      this.footstepData = footsteps;
      this.footstepOrder = footstepOrder;
      this.flag = flag;
   }
}
