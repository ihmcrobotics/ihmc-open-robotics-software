package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoInteger;

public class AverageYoFrameVector3D extends YoFrameVector3D
{
   private final YoInteger sampleSize;
   private final Tuple3DReadOnly dataSource;

   public AverageYoFrameVector3D(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, "", registry, referenceFrame, null);
   }

   public AverageYoFrameVector3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, nameSuffix, registry, referenceFrame, null);
   }

   public AverageYoFrameVector3D(String namePrefix, YoRegistry registry, FrameTuple3DReadOnly dataSource)
   {
      this(namePrefix, "", registry, dataSource.getReferenceFrame(), dataSource);
   }

   public AverageYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly dataSource)
   {
      this(namePrefix, nameSuffix, registry, dataSource.getReferenceFrame(), dataSource);
   }

   private AverageYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly dataSource)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.dataSource = dataSource;

      sampleSize = new YoInteger(YoGeometryNameTools.assembleName(namePrefix, "sampleSize", nameSuffix), registry);
   }

   public void update()
   {
      update(dataSource);
   }

   public void update(Tuple3DReadOnly vectorSource)
   {
      update(vectorSource.getX(), vectorSource.getY(), vectorSource.getZ());
   }

   public void update(FrameTuple3DReadOnly vectorSource)
   {
      checkReferenceFrameMatch(vectorSource);
      update((Tuple3DReadOnly) vectorSource);
   }

   public void update(double xSource, double ySource, double zSource)
   {
      sampleSize.increment();
      setX(getX() + (xSource - getX()) / sampleSize.getValue());
      setY(getY() + (ySource - getY()) / sampleSize.getValue());
      setZ(getZ() + (zSource - getZ()) / sampleSize.getValue());
   }

   public void reset()
   {
      sampleSize.set(0);
   }

   public int getSampleSize()
   {
      return sampleSize.getValue();
   }
}
