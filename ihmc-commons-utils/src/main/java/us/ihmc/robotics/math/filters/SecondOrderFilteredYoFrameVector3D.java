package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class SecondOrderFilteredYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final SecondOrderFilteredYoDouble x, y, z;

   public SecondOrderFilteredYoFrameVector3D(String namePrefix,
                                             String nameSuffix,
                                             YoRegistry registry,
                                             double dt,
                                             SecondOrderFilteredYoVariableParameters parameters,
                                             YoFrameVector3D unfilteredVector)
   {
      this(new SecondOrderFilteredYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoX()),
           new SecondOrderFilteredYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoY()),
           new SecondOrderFilteredYoDouble(YoGeometryNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoZ()),
           unfilteredVector.getReferenceFrame());
   }

   private SecondOrderFilteredYoFrameVector3D(SecondOrderFilteredYoDouble x,
                                              SecondOrderFilteredYoDouble y,
                                              SecondOrderFilteredYoDouble z,
                                              ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static SecondOrderFilteredYoFrameVector3D createSecondOrderFilteredYoFrameVector(String namePrefix,
                                                                                           String nameSuffix,
                                                                                           YoRegistry registry,
                                                                                           double dt,
                                                                                           double naturalFrequencyInHz,
                                                                                           double dampingRatio,
                                                                                           SecondOrderFilterType filterType,
                                                                                           ReferenceFrame referenceFrame)

   {
      SecondOrderFilteredYoVariableParameters parameters = new SecondOrderFilteredYoVariableParameters(namePrefix + nameSuffix,
                                                                                                       registry,
                                                                                                       naturalFrequencyInHz,
                                                                                                       dampingRatio,
                                                                                                       filterType);
      return createSecondOrderFilteredYoFrameVector(namePrefix, nameSuffix, registry, dt, parameters, referenceFrame);
   }

   public static SecondOrderFilteredYoFrameVector3D createSecondOrderFilteredYoFrameVector(String namePrefix,
                                                                                           String nameSuffix,
                                                                                           YoRegistry registry,
                                                                                           double dt,
                                                                                           SecondOrderFilteredYoVariableParameters parameters,
                                                                                           ReferenceFrame referenceFrame)
   {
      SecondOrderFilteredYoDouble x, y, z;
      x = new SecondOrderFilteredYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters);
      y = new SecondOrderFilteredYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters);
      z = new SecondOrderFilteredYoDouble(YoGeometryNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters);
      return new SecondOrderFilteredYoFrameVector3D(x, y, z, referenceFrame);
   }

   public static SecondOrderFilteredYoFrameVector3D createSecondOrderFilteredYoFrameVector(String namePrefix,
                                                                                           String nameSuffix,
                                                                                           YoRegistry registry,
                                                                                           double dt,
                                                                                           double naturalFrequencyInHz,
                                                                                           double dampingRatio,
                                                                                           SecondOrderFilterType filterType,
                                                                                           YoFrameVector3D unfilteredVector)
   {
      SecondOrderFilteredYoVariableParameters parameters = new SecondOrderFilteredYoVariableParameters(namePrefix + nameSuffix,
                                                                                                       registry,
                                                                                                       naturalFrequencyInHz,
                                                                                                       dampingRatio,
                                                                                                       filterType);
      return new SecondOrderFilteredYoFrameVector3D(namePrefix, nameSuffix, registry, dt, parameters, unfilteredVector);
   }

   @Override
   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
      z.update(zUnfiltered);
   }

   public void update(Vector3D vectorUnfiltered)
   {
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void update(FrameVector3D vectorUnfiltered)
   {
      checkReferenceFrameMatch(vectorUnfiltered);
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   @Override
   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
