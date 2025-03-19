package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.filters.DeadbandedYoVariable;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class DeadbandedYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final DeadbandedYoVariable x, y, z;

   private DeadbandedYoFrameVector3D(DeadbandedYoVariable x, DeadbandedYoVariable y, DeadbandedYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static DeadbandedYoFrameVector3D createDeadzoneYoFrameVector(String namePrefix, YoRegistry registry, YoDouble deadzoneSize, ReferenceFrame referenceFrame)
   {
      return createDeadzoneYoFrameVector(namePrefix, "", registry, deadzoneSize, referenceFrame);
   }

   public static DeadbandedYoFrameVector3D createDeadzoneYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry, YoDouble deadzoneSize, ReferenceFrame referenceFrame)
   {
      DeadbandedYoVariable x = new DeadbandedYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), deadzoneSize, registry);
      DeadbandedYoVariable y = new DeadbandedYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), deadzoneSize, registry);
      DeadbandedYoVariable z = new DeadbandedYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), deadzoneSize, registry);

      DeadbandedYoFrameVector3D ret = new DeadbandedYoFrameVector3D(x, y, z, referenceFrame);

      return ret;
   }

   public static DeadbandedYoFrameVector3D createDeadzoneYoFrameVector(String namePrefix, YoRegistry registry, YoDouble deadzoneSize, YoFrameTuple3D rawTuple)
   {
      return createDeadzoneYoFrameVector(namePrefix, "", registry, deadzoneSize, rawTuple);
   }

   public static DeadbandedYoFrameVector3D createDeadzoneYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry, YoDouble deadzoneSize, YoFrameTuple3D rawTuple)
   {
      DeadbandedYoVariable x = new DeadbandedYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), rawTuple.getYoX(), deadzoneSize, registry);
      DeadbandedYoVariable y = new DeadbandedYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), rawTuple.getYoY(), deadzoneSize, registry);
      DeadbandedYoVariable z = new DeadbandedYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), rawTuple.getYoZ(), deadzoneSize, registry);

      DeadbandedYoFrameVector3D ret = new DeadbandedYoFrameVector3D(x, y, z, rawTuple.getReferenceFrame());

      return ret;
   }

   @Override
   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(FrameTuple3DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);

      x.update(frameTuple.getX());
      y.update(frameTuple.getY());
      z.update(frameTuple.getZ());
   }
}
