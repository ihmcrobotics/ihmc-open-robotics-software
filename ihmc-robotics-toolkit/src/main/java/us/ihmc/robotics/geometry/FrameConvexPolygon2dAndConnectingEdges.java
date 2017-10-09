package us.ihmc.robotics.geometry;

public class FrameConvexPolygon2dAndConnectingEdges
{
   private final FrameConvexPolygon2d originalPolygon1, originalPolygon2;
   private final FrameConvexPolygon2d combinedPolygon;
   private final FrameLineSegment2d connectingEdge1, connectingEdge2;

   public FrameConvexPolygon2dAndConnectingEdges(FrameConvexPolygon2d originalPolygon1, FrameConvexPolygon2d originalPolygon2, FrameConvexPolygon2d polygon, FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2)
   {
      this.originalPolygon1 = originalPolygon1;
      this.originalPolygon2 = originalPolygon2;
      
      this.combinedPolygon = polygon;
      this.connectingEdge1 = connectingEdge1;
      this.connectingEdge2 = connectingEdge2;
   }

   public FrameConvexPolygon2d getFrameConvexPolygon2d()
   {
      return combinedPolygon;
   }

   public FrameLineSegment2d getConnectingEdge1()
   {
      return connectingEdge1;
   }

   public FrameLineSegment2d getConnectingEdge2()
   {
      return connectingEdge2;
   }

   public FrameConvexPolygon2d getOriginalPolygon1()
   {
      return originalPolygon1;
   }

   public FrameConvexPolygon2d getOriginalPolygon2()
   {
      return originalPolygon2;
   }
   
}
