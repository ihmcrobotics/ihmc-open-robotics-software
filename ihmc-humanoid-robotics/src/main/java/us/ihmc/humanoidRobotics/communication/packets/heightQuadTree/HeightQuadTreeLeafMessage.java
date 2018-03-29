package us.ihmc.humanoidRobotics.communication.packets.heightQuadTree;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple2D.Point2D32;

public class HeightQuadTreeLeafMessage extends Packet<HeightQuadTreeLeafMessage>
{
   public float height;
   public Point2D32 center = new Point2D32();

   public HeightQuadTreeLeafMessage()
   {
   }

   @Override
   public void set(HeightQuadTreeLeafMessage other)
   {
      height = other.height;
      center.set(other.center);
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeLeafMessage other, double epsilon)
   {
      return MathTools.epsilonCompare(height, other.height, epsilon) && center.epsilonEquals(other.center, epsilon);
   }
}
