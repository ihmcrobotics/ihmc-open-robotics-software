package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameScalableBoundingBox3d
{
   private final ReferenceFrame frame;
   private BoundingBox3D box;
   
   private final Point3D dimensions;
   private final Point3D center;
   
   // TODO make this whole class less garbage-full
   public FrameScalableBoundingBox3d(ReferenceFrame frame, Point3D dimensions, Point3D center) {
      this.frame = frame;
      this.dimensions = dimensions;
      this.center = center;
     
      setScale(1.0f);
   }
   
   public void setScale(float scale) {
      Point3D min = new Point3D(dimensions);
      Point3D max = new Point3D(dimensions);

      min.scaleAdd(-scale, center);
      max.scaleAdd(scale, center);

      this.box = new BoundingBox3D(min, max);
   }
   
   public boolean contains(double x, double y, double z) {
      return contains(new Point3D(x,y,z));
   }
   
   public boolean contains(Point3D p) {
      Point3D local = new Point3D(p);
      frame.getInverseTransformToRoot().transform(local);

      return box.isInsideInclusive(local);
   }
   
   public boolean intersects(Point3D start, Point3D end) {
      Point3D inFrameStart = new Point3D(start);
      frame.getInverseTransformToRoot().transform(inFrameStart);
      Point3D inFrameEnd = new Point3D(end);
      frame.getInverseTransformToRoot().transform(inFrameEnd);
      
      return box.doesIntersectWithLineSegment3D(start, end);
   }
}
