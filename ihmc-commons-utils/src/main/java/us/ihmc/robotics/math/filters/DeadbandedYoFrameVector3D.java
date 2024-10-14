package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class DeadbandedYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final DeadbandedYoVariable x, y, z;

   public DeadbandedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider deadzoneSize, YoFrameVector3D tupleToTrack)
   {
      this(new DeadbandedYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), tupleToTrack.getYoX(), deadzoneSize, registry),
           new DeadbandedYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), tupleToTrack.getYoY(), deadzoneSize, registry),
           new DeadbandedYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), tupleToTrack.getYoZ(), deadzoneSize, registry),
           tupleToTrack.getReferenceFrame());
   }

   public DeadbandedYoFrameVector3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider deadzoneSize, ReferenceFrame referenceFrame)
   {
      this(new DeadbandedYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), deadzoneSize, registry),
           new DeadbandedYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), deadzoneSize, registry),
           new DeadbandedYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), deadzoneSize, registry),
           referenceFrame);
   }

   private DeadbandedYoFrameVector3D(DeadbandedYoVariable x, DeadbandedYoVariable y, DeadbandedYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
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
