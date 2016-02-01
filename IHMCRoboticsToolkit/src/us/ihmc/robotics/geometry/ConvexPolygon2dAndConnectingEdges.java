package us.ihmc.robotics.geometry;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class ConvexPolygon2dAndConnectingEdges
{
   private final ConvexPolygon2d polygon;
   private final LineSegment2d connectingEdge1, connectingEdge2;

   public ConvexPolygon2dAndConnectingEdges(ConvexPolygon2d polygon, LineSegment2d connectingEdge1, LineSegment2d connectingEdge2)
   {
      this.polygon = polygon;
      this.connectingEdge1 = connectingEdge1;
      this.connectingEdge2 = connectingEdge2;
   }

   public ConvexPolygon2d getConvexPolygon2d()
   {
      return polygon;
   }

   public LineSegment2d getConnectingEdge1()
   {
      return connectingEdge1;
   }

   public LineSegment2d getConnectingEdge2()
   {
      return connectingEdge2;
   }

}
