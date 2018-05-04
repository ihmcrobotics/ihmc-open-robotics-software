package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class AlphaFilteredYoFramePoint extends YoFramePoint3D
{
   private final AlphaFilteredYoVariable x, y, z;

   private AlphaFilteredYoFramePoint(AlphaFilteredYoVariable x, AlphaFilteredYoVariable y, AlphaFilteredYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
           ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                           DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, referenceFrame);

      return ret;
   }


   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
           YoFramePoint3D unfilteredPoint)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoZ());

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());

      return ret;
   }


   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                           DoubleProvider alpha, YoFramePoint3D unfilteredPoint)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoZ());

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());

      return ret;
   }
   
   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
         double alphaX, double alphaY, double alphaZ, YoFramePoint3D unfilteredPoint)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alphaX, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alphaY, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alphaZ, unfilteredPoint.getYoZ());
      
      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());
      
      return ret;
   }
   
   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                           DoubleProvider alphaX, DoubleProvider alphaY, DoubleProvider alphaZ, YoFramePoint3D unfilteredPoint)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alphaX, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alphaY, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alphaZ, unfilteredPoint.getYoZ());
      
      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());
      
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

   public void update(Point3D pointUnfiltered)
   {
      x.update(pointUnfiltered.getX());
      y.update(pointUnfiltered.getY());
      z.update(pointUnfiltered.getZ());
   }

   public void update(FramePoint3D pointUnfiltered)
   {
      checkReferenceFrameMatch(pointUnfiltered);
      x.update(pointUnfiltered.getX());
      y.update(pointUnfiltered.getY());
      z.update(pointUnfiltered.getZ());
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }
}
