package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;

public class FrameScalableBoundingBox3d
{
   private final ReferenceFrame frame;
   private BoundingBox3d box;
   
   private final Point3d dimensions;
   private final Point3d center;
   
   // TODO make this whole class less garbage-full
   public FrameScalableBoundingBox3d(ReferenceFrame frame, Point3d dimensions, Point3d center) {
      this.frame = frame;
      this.dimensions = dimensions;
      this.center = center;
     
      setScale(1.0f);
   }
   
   public void setScale(float scale) {
      Point3d min = new Point3d(dimensions);
      Point3d max = new Point3d(dimensions);

      min.scaleAdd(-scale, center);
      max.scaleAdd(scale, center);

      this.box = new BoundingBox3d(min, max);
   }
   
   public boolean contains(double x, double y, double z) {
      return contains(new Point3d(x,y,z));
   }
   
   public boolean contains(Point3d p) {
      Point3d local = new Point3d(p);
      frame.getInverseTransformToRoot().transform(local);

      return box.isInside(local);
   }
   
   public boolean intersects(Point3d start, Point3d end) {
      Point3d inFrameStart = new Point3d(start);
      frame.getInverseTransformToRoot().transform(inFrameStart);
      Point3d inFrameEnd = new Point3d(end);
      frame.getInverseTransformToRoot().transform(inFrameEnd);
      
      return box.intersects(inFrameStart, inFrameEnd);
   }
}
