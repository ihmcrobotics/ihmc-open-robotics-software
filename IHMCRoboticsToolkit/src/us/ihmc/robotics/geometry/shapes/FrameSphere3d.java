package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSphere3d extends FrameShape3d<FrameSphere3d, Sphere3d>
{
   private final Sphere3d sphere;
   
   public FrameSphere3d()
   {
      super(new Sphere3d());
      sphere = getGeometryObject();
   }
   
   public FrameSphere3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Sphere3d());
      sphere = getGeometryObject();
   }
   
   public FrameSphere3d(ReferenceFrame referenceFrame, double x, double y, double z, double radius)
   {
      super(referenceFrame, new Sphere3d(x, y, z, radius));
      sphere = getGeometryObject();
   }
   
   public double getRadius()
   {
      return sphere.getRadius();
   }
}
