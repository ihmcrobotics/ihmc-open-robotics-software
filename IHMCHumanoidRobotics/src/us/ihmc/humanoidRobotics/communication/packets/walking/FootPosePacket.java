package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller to move the desired foot to a given location\n"
                                  + "and orientation in world coordinates. This message is not intended to specify footsteps\n"
                                  + "as the controller will use a straight-line trajectory when moving the foot from its\n"
                                  + "current to desired location/orientation")
public class FootPosePacket extends Packet<FootPosePacket> implements TransformableDataObject<FootPosePacket>, VisualizablePacket
{

   public RobotSide robotSide;

   public Point3d position;
   public Quat4d orientation;
   @FieldDocumentation("trajectoryTime specifies how fast or how slow to move to the desired pose")
   public double trajectoryTime;

   public FootPosePacket()
   {
      // Empty constructor for deserialization
   }
   
   public FootPosePacket(FootPosePacket footPosePacket)
   {
      this(footPosePacket.robotSide, footPosePacket.position, footPosePacket.orientation, footPosePacket.trajectoryTime); 
   }

   public FootPosePacket(RobotSide robotSide, Point3d position, Quat4d orientation, double trajectoryTime)
   {
      this.robotSide = robotSide;
      this.position = position;
      this.orientation = orientation;
      this.trajectoryTime = trajectoryTime;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public FootPosePacket transform(RigidBodyTransform transform)
   {
      FootPosePacket ret = new FootPosePacket();

      // RobotSide robotSide;
      ret.robotSide = this.getRobotSide();

      // Point3d position;
      ret.position = TransformTools.getTransformedPoint(this.getPosition(), transform);

      // Quat4d orientation;
      ret.orientation = TransformTools.getTransformedQuat(this.getOrientation(), transform);

      ret.trajectoryTime = this.trajectoryTime;

      return ret;
   }

   public String toString()
   {
      return robotSide.getSideNameFirstLetter() + " Foot Pose";
   }

   @Override
   public boolean epsilonEquals(FootPosePacket other, double epsilon)
   {
      boolean sameRobotSide = this.getRobotSide().equals(other.getRobotSide());
      boolean sameOrientation = RotationTools.quaternionEpsilonEquals(this.getOrientation(), other.getOrientation(), epsilon);
      boolean samePosition = this.getPosition().epsilonEquals(other.getPosition(), epsilon);
      boolean sameTrajectoryTime = MathTools.epsilonEquals(this.trajectoryTime, other.trajectoryTime,  epsilon);

      return sameRobotSide && sameOrientation && samePosition && sameTrajectoryTime;
   }

   public FootPosePacket(Random random)
   {
      this.robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      this.position = RandomTools.generateRandomPointWithEdgeCases(random, 0.05);
      this.orientation = RandomTools.generateRandomQuaternion(random);
      this.trajectoryTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1);
   }
}
