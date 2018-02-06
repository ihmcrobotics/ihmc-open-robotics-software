package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FrameConvexPolygon2dAndConnectingEdges
{
   private final FrameConvexPolygon2d originalPolygon1, originalPolygon2;
   private final FrameConvexPolygon2d combinedPolygon;
   private final FrameLineSegment2D connectingEdge1, connectingEdge2;

   public FrameConvexPolygon2dAndConnectingEdges()
   {
      this.originalPolygon1 = new FrameConvexPolygon2d();
      this.originalPolygon2 = new FrameConvexPolygon2d();

      this.combinedPolygon = new FrameConvexPolygon2d();
      this.connectingEdge1 = new FrameLineSegment2D();
      this.connectingEdge2 = new FrameLineSegment2D();
   }

   public FrameConvexPolygon2dAndConnectingEdges(FrameConvexPolygon2d originalPolygon1, FrameConvexPolygon2d originalPolygon2, FrameConvexPolygon2d polygon, FrameLineSegment2D connectingEdge1, FrameLineSegment2D connectingEdge2)
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

   public FrameLineSegment2D getConnectingEdge1()
   {
      return connectingEdge1;
   }

   public FrameLineSegment2D getConnectingEdge2()
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

   public void setIncludingFrameAndUpdate(FrameConvexPolygon2d originalPolygon1, FrameConvexPolygon2d originalPolygon2, FrameConvexPolygon2d combinedPolygon, FrameLineSegment2D connectingEdge1,
         FrameLineSegment2D connectingEdge2)
   {
      this.originalPolygon1.setIncludingFrameAndUpdate(originalPolygon1);
      this.originalPolygon2.setIncludingFrameAndUpdate(originalPolygon2);
      this.combinedPolygon.setIncludingFrameAndUpdate(combinedPolygon);
      this.connectingEdge1.setIncludingFrame(connectingEdge1);
      this.connectingEdge2.setIncludingFrame(connectingEdge2);
   }

   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, FrameConvexPolygon2d originalPolygon1, FrameConvexPolygon2d originalPolygon2,
         ConvexPolygon2D combinedPolygon, LineSegment2D connectingEdge1, LineSegment2D connectingEdge2)
   {
      this.originalPolygon1.setIncludingFrameAndUpdate(originalPolygon1);
      this.originalPolygon2.setIncludingFrameAndUpdate(originalPolygon2);
      this.combinedPolygon.setIncludingFrameAndUpdate(referenceFrame, combinedPolygon);
      this.connectingEdge1.setIncludingFrame(referenceFrame, connectingEdge1);
      this.connectingEdge2.setIncludingFrame(referenceFrame, connectingEdge2);
   }
}
