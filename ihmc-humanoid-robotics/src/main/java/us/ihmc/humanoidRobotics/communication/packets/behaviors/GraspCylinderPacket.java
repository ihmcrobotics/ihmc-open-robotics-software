package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;

public class GraspCylinderPacket extends Packet<GraspCylinderPacket>
{
   public RobotSide robotSide;
   public Point3D graspPointInWorld;
   public Vector3D cylinderLongAxisInWorld;

   public GraspCylinderPacket(Random random)
   {
      robotSide = RobotSide.generateRandomRobotSide(random);
      graspPointInWorld = RandomGeometry.nextPoint3D(random, -10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
      cylinderLongAxisInWorld = RandomGeometry.nextVector3D(random);
   }
   
   public GraspCylinderPacket()
   {
   }

   public GraspCylinderPacket(RobotSide robotSide, Point3D graspPointInWorld, Vector3D cylinderLongAxisInWorld)
   {
      this.robotSide = robotSide;
      this.graspPointInWorld = graspPointInWorld;
      this.cylinderLongAxisInWorld = cylinderLongAxisInWorld;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Point3D getGraspPointInWorld()
   {
      return graspPointInWorld;
   }

   public Vector3D getCylinderLongAxisInWorld()
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
