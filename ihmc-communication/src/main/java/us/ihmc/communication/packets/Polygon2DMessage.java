package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.idl.PreallocatedList;

public class Polygon2DMessage extends Packet<Polygon2DMessage>
{
   public PreallocatedList<Point2D32> vertices = new PreallocatedList<>(Point2D32.class, Point2D32::new, 100);

   public Polygon2DMessage()
   {
   }

   @Override
   public void set(Polygon2DMessage other)
   {
      MessageTools.copyData(other.vertices, vertices);
   }

   public PreallocatedList<Point2D32> getVertices()
   {
      return vertices;
   }

   @Override
   public boolean epsilonEquals(Polygon2DMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(vertices, other.vertices, epsilon);
   }
}
