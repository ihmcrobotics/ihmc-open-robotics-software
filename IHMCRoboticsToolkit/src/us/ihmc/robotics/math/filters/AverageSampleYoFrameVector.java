package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class AverageSampleYoFrameVector extends YoFrameVector
{
   private final AverageSampleDoubleYoVariable x, y, z;
   
   private AverageSampleYoFrameVector(AverageSampleDoubleYoVariable x, AverageSampleDoubleYoVariable y, AverageSampleDoubleYoVariable z, ReferenceFrame referenceFrame)
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
      AverageSampleDoubleYoVariable x = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      AverageSampleDoubleYoVariable y = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      AverageSampleDoubleYoVariable z = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      
      AverageSampleYoFrameVector ret = new AverageSampleYoFrameVector(x, y, z, referenceFrame);
      
      return ret;
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, YoVariableRegistry registry, YoFrameVector dataSource, ReferenceFrame referenceFrame)
   {
      return createAverageSampleYoFrameVector(namePrefix, "", registry, dataSource, referenceFrame);
   }

   public static AverageSampleYoFrameVector createAverageSampleYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, YoFrameVector dataSource, ReferenceFrame referenceFrame)
   {
      AverageSampleDoubleYoVariable x = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), dataSource.getYoX(), registry);
      AverageSampleDoubleYoVariable y = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), dataSource.getYoY(), registry);
      AverageSampleDoubleYoVariable z = new AverageSampleDoubleYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), dataSource.getYoZ(), registry);
      
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

   public void update(FrameVector vectorSource)
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
