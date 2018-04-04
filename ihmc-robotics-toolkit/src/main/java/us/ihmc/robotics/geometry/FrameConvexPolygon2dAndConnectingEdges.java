package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;

public class FrameConvexPolygon2dAndConnectingEdges
{
   private final FrameConvexPolygon2D originalPolygon1 = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D originalPolygon2 = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D combinedPolygon = new FrameConvexPolygon2D();
   private final FrameLineSegment2D connectingEdge1 = new FrameLineSegment2D();
   private final FrameLineSegment2D connectingEdge2 = new FrameLineSegment2D();

   public FrameConvexPolygon2dAndConnectingEdges()
   {
   }

   public FrameConvexPolygon2dAndConnectingEdges(FrameConvexPolygon2DReadOnly originalPolygon1, FrameConvexPolygon2DReadOnly originalPolygon2,
                                                 FrameConvexPolygon2DReadOnly polygon, FrameLineSegment2DReadOnly connectingEdge1,
                                                 FrameLineSegment2DReadOnly connectingEdge2)
   {
      setIncludingFrame(originalPolygon1, originalPolygon2, polygon, connectingEdge1, connectingEdge2);
   }

   public FrameConvexPolygon2D getFrameConvexPolygon2d()
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

   public FrameConvexPolygon2D getOriginalPolygon1()
   {
      return originalPolygon1;
   }

   public FrameConvexPolygon2D getOriginalPolygon2()
   {
      return originalPolygon2;
   }

   public void setIncludingFrame(FrameConvexPolygon2DReadOnly originalPolygon1, FrameConvexPolygon2DReadOnly originalPolygon2,
                                 FrameConvexPolygon2DReadOnly combinedPolygon, FrameLineSegment2DReadOnly connectingEdge1,
                                 FrameLineSegment2DReadOnly connectingEdge2)
   {
      this.originalPolygon1.setIncludingFrame(originalPolygon1);
      this.originalPolygon2.setIncludingFrame(originalPolygon2);
      this.combinedPolygon.setIncludingFrame(combinedPolygon);
      this.connectingEdge1.setIncludingFrame(connectingEdge1);
      this.connectingEdge2.setIncludingFrame(connectingEdge2);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, FrameConvexPolygon2DReadOnly originalPolygon1, FrameConvexPolygon2DReadOnly originalPolygon2,
                                 ConvexPolygon2DReadOnly combinedPolygon, LineSegment2DReadOnly connectingEdge1, LineSegment2DReadOnly connectingEdge2)
   {
      this.originalPolygon1.setIncludingFrame(originalPolygon1);
      this.originalPolygon2.setIncludingFrame(originalPolygon2);
      this.combinedPolygon.setIncludingFrame(referenceFrame, combinedPolygon);
      this.connectingEdge1.setIncludingFrame(referenceFrame, connectingEdge1);
      this.connectingEdge2.setIncludingFrame(referenceFrame, connectingEdge2);
   }
}
