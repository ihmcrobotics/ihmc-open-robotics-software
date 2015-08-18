package us.ihmc.communication.packets.sensing;

import java.util.Random;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

@ClassDocumentation(documentation = "This message specifies a point in world coordinates for the robot to look at.\n"
                                  + "The controller will attempt to orient the head such that the point is within\n"
                                  + "the FOV of the camera using a combination of the neck and pelvis joints. If the\n"
                                  + "point is outside the combined range of the available joints, the robot will\n"
                                  + "turn as close as it can to the point without taking any steps.")
public class LookAtPacket extends IHMCRosApiPacket<LookAtPacket>
{
   public Point3d lookAtPoint;
   public int index;

   public LookAtPacket()
   {
   }

   public LookAtPacket(Point3d lookAtPoint, int index)
   {
      super();
      this.lookAtPoint = lookAtPoint;
      this.index = index;
   }

   public Point3d getLookAtPoint()
   {
      return lookAtPoint;
   }

   public LookAtPacket transform(RigidBodyTransform transform, int index)
   {
      Point3d lookAtPoint = new Point3d(this.getLookAtPoint());
      transform.transform(lookAtPoint);
      LookAtPacket ret = new LookAtPacket(lookAtPoint, index);

      return ret;
   }

   public boolean epsilonEquals(LookAtPacket other, double epsilon)
   {
      return this.getLookAtPoint().epsilonEquals(other.getLookAtPoint(), epsilon);
   }

   public LookAtPacket(Random random)
   {
      this(RandomTools.generateRandomPoint(random, -0.5, -1.0, -1.0, 0.75, 1.0, 1.0), random.nextInt());
   }

   public int getIndex()
   {
      return index;
   }
}
