package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class AlphaFilteredYoFrameVector extends YoFrameVector3D implements ProcessingYoVariable
{
   private final AlphaFilteredYoVariable x, y, z;

   private AlphaFilteredYoFrameVector(AlphaFilteredYoVariable x, AlphaFilteredYoVariable y, AlphaFilteredYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static AlphaFilteredYoFrameVector createAlphaFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
           ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFrameVector ret = new AlphaFilteredYoFrameVector(x, y, z, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFrameVector createAlphaFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFrameVector ret = new AlphaFilteredYoFrameVector(x, y, z, referenceFrame);

      return ret;
   }


   public static AlphaFilteredYoFrameVector createAlphaFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
           YoFrameVector3D unfilteredVector)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoZ());

      AlphaFilteredYoFrameVector ret = new AlphaFilteredYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());

      return ret;
   }


   public static AlphaFilteredYoFrameVector createAlphaFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                             DoubleProvider alpha, YoFrameVector3D unfilteredVector)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoZ());

      AlphaFilteredYoFrameVector ret = new AlphaFilteredYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());

      return ret;
   }

   public static AlphaFilteredYoFrameVector createAlphaFilteredYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                             DoubleProvider alpha, YoFramePoint3D unfilteredPosition)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPosition.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPosition.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredPosition.getYoZ());

      AlphaFilteredYoFrameVector ret = new AlphaFilteredYoFrameVector(x, y, z, unfilteredPosition.getReferenceFrame());

      return ret;
   }

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

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
