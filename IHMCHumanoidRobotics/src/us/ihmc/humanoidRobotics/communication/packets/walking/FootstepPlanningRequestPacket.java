package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket>
{
   public RobotSide initialStanceSide;
   public Point3f stanceFootPositionInWorld;
   public Quat4f stanceFootOrientationInWorld;
   public Point3f goalPositionInWorld;
   public Quat4f goalOrientationInWorld;
   public boolean assumeFlatGround = true;

   public FootstepPlanningRequestPacket()
   {
      // empty constructor for serialization
   }

   public FootstepPlanningRequestPacket(FramePose initialStanceFootPose, RobotSide initialStanceSide, FramePose goalPose)
   {
      set(initialStanceFootPose, initialStanceSide, goalPose);
   }

   public void set(FramePose initialStanceFootPose, RobotSide initialStanceSide, FramePose goalPose)
   {
      this.initialStanceSide = initialStanceSide;

      Point3d tempPoint = new Point3d();
      Quat4d tempOrientation = new Quat4d();

      initialStanceFootPose.getPosition(tempPoint);
      initialStanceFootPose.getOrientation(tempOrientation);
      stanceFootPositionInWorld = new Point3f(tempPoint);
      stanceFootOrientationInWorld = new Quat4f(tempOrientation);

      goalPose.getPosition(tempPoint);
      goalPose.getOrientation(tempOrientation);
      goalPositionInWorld = new Point3f(tempPoint);
      goalOrientationInWorld = new Quat4f(tempOrientation);
   }

   public void setAssumeFlatGround(boolean assumeFlatGround)
   {
      this.assumeFlatGround = assumeFlatGround;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningRequestPacket other, double epsilon)
   {
      if (!initialStanceSide.equals(other.initialStanceSide))
         return false;
      if (!stanceFootPositionInWorld.epsilonEquals(other.stanceFootPositionInWorld, (float) epsilon))
         return false;
      if (!RotationTools.quaternionEpsilonEquals(stanceFootOrientationInWorld, other.stanceFootOrientationInWorld, (float) epsilon))
         return false;
      if (!goalPositionInWorld.epsilonEquals(other.goalPositionInWorld, (float) epsilon))
         return false;
      if (!RotationTools.quaternionEpsilonEquals(goalOrientationInWorld, other.goalOrientationInWorld, (float) epsilon))
         return false;
      return true;
   }

}
