package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class SnapFootstepPacket extends Packet<SnapFootstepPacket>
{
   //   public static final byte UNKOWN = 0;
   //   public static final byte VALID_UNCHANGED_STEP = 1;
   //   public static final byte VALID_SNAPPED_STEP = 2;
   //   public static final byte BAD_STEP = 3;
   public ArrayList<FootstepData> footstepData;
   public int[] footstepOrder;
   public byte[] flag;

   public SnapFootstepPacket()
   {
      // Empty constructor for deserialization
   }

   public SnapFootstepPacket(ArrayList<FootstepData> data, int[] footstepOrder, byte[] flag)
   {
      this.footstepData = data;
      this.footstepOrder = footstepOrder;
      this.flag = flag;
   }

   public ArrayList<FootstepData> getFootstepData()
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
      ArrayList<FootstepData> footsteps = new ArrayList<FootstepData>();
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

         Point3d position = RandomTools.generateRandomPoint(random, xMax, yMax, zMax);

         Quat4d orientation = new Quat4d();
         orientation.set(RandomTools.generateRandomRotation(random));

         previousFootstep.transform(position);

         previousFootstep.setTranslation(new Vector3f(position));
         previousFootstep.setRotation(orientation);

         FootstepData footstepData = new FootstepData(robotSide, new Point3d(position), orientation);

         footsteps.add(footstepData);
      }

      this.footstepData = footsteps;
      this.footstepOrder = footstepOrder;
      this.flag = flag;
   }
}
