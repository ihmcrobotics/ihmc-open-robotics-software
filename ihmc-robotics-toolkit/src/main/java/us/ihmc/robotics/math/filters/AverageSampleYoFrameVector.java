package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class AverageSampleYoFrameVector extends YoFrameVector
{
   private final AverageSampleYoDouble x, y, z;
   
   private AverageSampleYoFrameVector(AverageSampleYoDouble x, AverageSampleYoDouble y, AverageSampleYoDouble z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      return createAverageSampleYoFrameVector(namePrefix, "", registry, referenceFrame);
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      AverageSampleYoDouble x = new AverageSampleYoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      AverageSampleYoDouble y = new AverageSampleYoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      AverageSampleYoDouble z = new AverageSampleYoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      
      AverageSampleYoFrameVector ret = new AverageSampleYoFrameVector(x, y, z, referenceFrame);
      
      return ret;
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, YoVariableRegistry registry, YoFrameVector dataSource, ReferenceFrame referenceFrame)
   {
      return createAverageSampleYoFrameVector(namePrefix, "", registry, dataSource, referenceFrame);
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoFrameVector dataSource, ReferenceFrame referenceFrame)
   {
      AverageSampleYoDouble x = new AverageSampleYoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), dataSource.getYoX(), registry);
      AverageSampleYoDouble y = new AverageSampleYoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), dataSource.getYoY(), registry);
      AverageSampleYoDouble z = new AverageSampleYoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), dataSource.getYoZ(), registry);
      
      AverageSampleYoFrameVector ret = new AverageSampleYoFrameVector(x, y, z, referenceFrame);
      
      return ret;
   }

   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xSource, double ySource, double zSource)
   {
      x.update(xSource);
      y.update(ySource);
      z.update(zSource);
   }

   public void update(Vector3D vectorSource)
   {
      x.update(vectorSource.getX());
      y.update(vectorSource.getY());
      z.update(vectorSource.getZ());
   }

   public void update(FrameVector3D vectorSource)
   {
      checkReferenceFrameMatch(vectorSource);
      x.update(vectorSource.getX());
      y.update(vectorSource.getY());
      z.update(vectorSource.getZ());
   }

   public void doAverage()
   {
      x.doAverage();
      y.doAverage();
      z.doAverage();
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
