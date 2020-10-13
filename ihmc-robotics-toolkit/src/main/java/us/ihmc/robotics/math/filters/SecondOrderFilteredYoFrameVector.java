package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class SecondOrderFilteredYoFrameVector extends YoFrameVector3D implements ProcessingYoVariable
{
   private final SecondOrderFilteredYoVariable x, y, z;

   private SecondOrderFilteredYoFrameVector(SecondOrderFilteredYoVariable x, SecondOrderFilteredYoVariable y, SecondOrderFilteredYoVariable z,
         ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static SecondOrderFilteredYoFrameVector createSecondOrderFilteredYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry,
         double dt, double naturalFrequencyInHz, double dampingRatio, SecondOrderFilterType filterType, ReferenceFrame referenceFrame)
   {
      SecondOrderFilteredYoVariableParameters parameters = new SecondOrderFilteredYoVariableParameters(namePrefix + nameSuffix, registry, naturalFrequencyInHz,
            dampingRatio, filterType);
      return createSecondOrderFilteredYoFrameVector(namePrefix, nameSuffix, registry, dt, parameters, referenceFrame);
   }

   public static SecondOrderFilteredYoFrameVector createSecondOrderFilteredYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry,
         double dt, SecondOrderFilteredYoVariableParameters parameters, ReferenceFrame referenceFrame)
   {
      SecondOrderFilteredYoVariable x, y, z;
      x = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters);
      y = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters);
      z = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters);
      return new SecondOrderFilteredYoFrameVector(x, y, z, referenceFrame);
   }

   public static SecondOrderFilteredYoFrameVector createSecondOrderFilteredYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry,
         double dt, double naturalFrequencyInHz, double dampingRatio, SecondOrderFilterType filterType, YoFrameVector3D unfilteredVector)
   {
      SecondOrderFilteredYoVariableParameters parameters = new SecondOrderFilteredYoVariableParameters(namePrefix + nameSuffix, registry, naturalFrequencyInHz,
            dampingRatio, filterType);
      return createSecondOrderFilteredYoFrameVector(namePrefix, nameSuffix, registry, dt, parameters, unfilteredVector);
   }

   public static SecondOrderFilteredYoFrameVector createSecondOrderFilteredYoFrameVector(String namePrefix, String nameSuffix, YoRegistry registry,
         double dt, SecondOrderFilteredYoVariableParameters parameters, YoFrameVector3D unfilteredVector)
   {
      SecondOrderFilteredYoVariable x, y, z;
      x = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoX());
      y = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoY());
      z = new SecondOrderFilteredYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), registry, dt, parameters, unfilteredVector.getYoZ());
      return new SecondOrderFilteredYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());
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
