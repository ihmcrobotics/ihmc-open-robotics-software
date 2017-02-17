package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket>
{
   public RobotSide initialStanceSide;
   public Point3D32 stanceFootPositionInWorld;
   public Quaternion32 stanceFootOrientationInWorld;
   public Point3D32 goalPositionInWorld;
   public Quaternion32 goalOrientationInWorld;
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

      Point3D tempPoint = new Point3D();
      Quaternion tempOrientation = new Quaternion();

      initialStanceFootPose.getPosition(tempPoint);
      initialStanceFootPose.getOrientation(tempOrientation);
      stanceFootPositionInWorld = new Point3D32(tempPoint);
      stanceFootOrientationInWorld = new Quaternion32(tempOrientation);

      goalPose.getPosition(tempPoint);
      goalPose.getOrientation(tempOrientation);
      goalPositionInWorld = new Point3D32(tempPoint);
      goalOrientationInWorld = new Quaternion32(tempOrientation);
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
