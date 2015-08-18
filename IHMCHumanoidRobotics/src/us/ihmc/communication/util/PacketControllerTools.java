package us.ihmc.communication.util;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * The purpose of this class is to provide a tool to easily create packets
 * for the controller
 */
public class PacketControllerTools
{
   public static HandPosePacket createGoToHomeHandPosePacket(RobotSide robotSide, double trajectoryTime)
   {
      HandPosePacket handPosePacket = new HandPosePacket();
      handPosePacket.robotSide = robotSide;
      handPosePacket.toHomePosition = true;
      handPosePacket.referenceFrame = Frame.WORLD;
      handPosePacket.jointAngles = null;
      handPosePacket.position = null;
      handPosePacket.orientation = null;
      handPosePacket.trajectoryTime = trajectoryTime;

      return handPosePacket;
   }
   
   public static HandPosePacket createHandPosePacket(Frame holdPoseInThisFrameIfRobotMoves, RigidBodyTransform transformToWorld, RobotSide robotSide, double trajectoryTime)
   {
      Vector3d translation = new Vector3d();
      Quat4d rotation = new Quat4d();
      transformToWorld.get(translation);
      transformToWorld.get(rotation);
      Point3d point = new Point3d(translation.getX(), translation.getY(), translation.getZ());
      HandPosePacket handPosePacket = new HandPosePacket(robotSide, holdPoseInThisFrameIfRobotMoves, point, rotation, trajectoryTime);
      return handPosePacket;
   }
   
   public static HandPosePacket createHandPosePacket(Frame holdPoseInThisFrameIfRobotMoves, Point3d position, Quat4d orientation, RobotSide robotSide, double trajectoryTime)
   {
      HandPosePacket handPosePacket = new HandPosePacket(robotSide, holdPoseInThisFrameIfRobotMoves, position, orientation, trajectoryTime);
      return handPosePacket;
   }

   public static PelvisPosePacket createPelvisPosePacketForPositionAndOrientation(Point3d position, Quat4d orientation, double trajectoryTime)
   {
      return new PelvisPosePacket(position, orientation, false, trajectoryTime);
   }

   public static PelvisPosePacket createPelvisPosePacketForOrientationOnly(Quat4d orientation, double trajectoryTime)
   {
      return new PelvisPosePacket(null, orientation, false, trajectoryTime);
   }

   public static PelvisPosePacket createPelvisPosePacketForPositionOnly(Point3d position, double trajectoryTime)
   {
      return new PelvisPosePacket(position, null, false, trajectoryTime);
   }

   public static PelvisPosePacket createGoToHomePelvisPosePacket(double trajectoryTime)
   {
      return new PelvisPosePacket(null, null, true, trajectoryTime);
   }

   public static ChestOrientationPacket createChestOrientationPacket(Quat4d orientation, double trajectoryTime)
   {
      return new ChestOrientationPacket(orientation, false, trajectoryTime);
   }

   public static ChestOrientationPacket createGoToHomeChestOrientationPacket(double trajectoryTime)
   {
      return new ChestOrientationPacket(null, true, trajectoryTime);
   }

}
