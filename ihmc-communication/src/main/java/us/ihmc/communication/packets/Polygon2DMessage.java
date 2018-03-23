package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class Polygon2DMessage extends Packet<Polygon2DMessage>
{
   public RecyclingArrayListPubSub<Point2D32> vertices = new RecyclingArrayListPubSub<>(Point2D32.class, Point2D32::new, 100);

   public Polygon2DMessage()
   {
   }

   @Override
   public void set(Polygon2DMessage other)
   {
      MessageTools.copyData(other.vertices, vertices);
   }

   public RecyclingArrayListPubSub<Point2D32> getVertices()
   {
      return vertices;
   }

   @Override
   public boolean epsilonEquals(Polygon2DMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(vertices, other.vertices, epsilon);
   }
}
