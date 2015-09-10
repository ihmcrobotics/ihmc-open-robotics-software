package us.ihmc.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

public class GraspCylinderPacket extends Packet<GraspCylinderPacket>
{
   public RobotSide robotSide;
   public Point3d graspPointInWorld;
   public Vector3d cylinderLongAxisInWorld;

   public GraspCylinderPacket(Random random)
   {
      robotSide = RobotSide.generateRandomRobotSide(random);
      graspPointInWorld = RandomTools.generateRandomPoint(random, -10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
      cylinderLongAxisInWorld = RandomTools.generateRandomVector(random);
   }
   
   public GraspCylinderPacket()
   {
   }

   public GraspCylinderPacket(RobotSide robotSide, Point3d graspPointInWorld, Vector3d cylinderLongAxisInWorld)
   {
      this.robotSide = robotSide;
      this.graspPointInWorld = graspPointInWorld;
      this.cylinderLongAxisInWorld = cylinderLongAxisInWorld;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Point3d getGraspPointInWorld()
   {
      return graspPointInWorld;
   }

   public Vector3d getCylinderLongAxisInWorld()
   {
      return cylinderLongAxisInWorld;
   }
   
   public boolean epsilonEquals(GraspCylinderPacket turnValvePacket, double epsilon)
   {
      boolean robotSideEquals = robotSide == turnValvePacket.robotSide;
      boolean graspPointEquals = graspPointInWorld == turnValvePacket.graspPointInWorld;
      boolean cylinderAxisEquals = cylinderLongAxisInWorld == turnValvePacket.cylinderLongAxisInWorld;
      
      return robotSideEquals && graspPointEquals && cylinderAxisEquals;
   }
}
