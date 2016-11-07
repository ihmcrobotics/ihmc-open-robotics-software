package us.ihmc.communication.packets;

import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanningToolboxOutputStatus extends StatusPacket<FootstepPlanningToolboxOutputStatus>
{
   public RobotSide firstStepSide;
   public Point3f[] footstepPositionsInWorld;
   public Quat4f[] footstepOrientationsInWorld;

   public FootstepPlanningToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public FootstepPlanningToolboxOutputStatus(Random random)
   {
      firstStepSide = RobotSide.generateRandomRobotSide(random);
      int steps = Math.abs(random.nextInt(1000));
      footstepPositionsInWorld = new Point3f[steps];
      footstepOrientationsInWorld = new Quat4f[steps];
      for (int i = 0; i < steps; i++)
      {
         footstepPositionsInWorld[i] = new Point3f(RandomTools.generateRandomVector(random));
         footstepOrientationsInWorld[i] = RandomTools.generateRandomQuaternion4f(random);
      }
   }

   public FootstepPlanningToolboxOutputStatus(List<FramePose> footstepPosesInWorld, RobotSide firstStepSide)
   {
      set(footstepPosesInWorld, firstStepSide);
   }

   public void set(List<FramePose> footstepPosesInWorld, RobotSide firstStepSide)
   {
      this.firstStepSide = firstStepSide;
      int steps = footstepPosesInWorld.size();
      footstepPositionsInWorld = new Point3f[steps];
      footstepOrientationsInWorld = new Quat4f[steps];

      Point3d tempPoint = new Point3d();
      Quat4d tempOrientation = new Quat4d();

      for (int i = 0; i < steps; i++)
      {
         FramePose footstepPose = footstepPosesInWorld.get(i);
         footstepPose.getPosition(tempPoint);
         footstepPose.getOrientation(tempOrientation);
         footstepPositionsInWorld[i] = new Point3f(tempPoint);
         footstepOrientationsInWorld[i] = new Quat4f(tempOrientation);
      }
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if (!firstStepSide.equals(other.firstStepSide))
         return false;
      if (!(footstepPositionsInWorld.length == other.footstepPositionsInWorld.length))
         return false;
      if (!(footstepOrientationsInWorld.length == other.footstepOrientationsInWorld.length))
         return false;

      for (int i = 0; i < footstepPositionsInWorld.length; i++)
      {
         if (!footstepPositionsInWorld[i].epsilonEquals(other.footstepPositionsInWorld[i], (float) epsilon))
            return false;
      }
      for (int i = 0; i < footstepOrientationsInWorld.length; i++)
      {
         if (!RotationTools.quaternionEpsilonEquals(footstepOrientationsInWorld[i], other.footstepOrientationsInWorld[i], (float) epsilon))
            return false;
      }

      return true;
   }

   @Override
   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      firstStepSide = other.firstStepSide;

      int positionsLength = other.footstepPositionsInWorld.length;
      footstepPositionsInWorld = new Point3f[positionsLength];
      System.arraycopy(other.footstepPositionsInWorld, 0, footstepPositionsInWorld, 0, positionsLength);

      int orientationsLength = other.footstepOrientationsInWorld.length;
      footstepOrientationsInWorld = new Quat4f[orientationsLength];
      System.arraycopy(other.footstepOrientationsInWorld, 0, footstepOrientationsInWorld, 0, orientationsLength);
   }

}
