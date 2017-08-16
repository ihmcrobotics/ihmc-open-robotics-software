package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class AlphaFilteredYoFramePoint extends YoFramePoint
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
           YoDouble alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, referenceFrame);

      return ret;
   }


   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
           YoFramePoint unfilteredPoint)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoZ());

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());

      return ret;
   }


   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
           YoDouble alpha, YoFramePoint unfilteredPoint)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoZ());

      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());

      return ret;
   }
   
   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
         double alphaX, double alphaY, double alphaZ, YoFramePoint unfilteredPoint)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alphaX, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alphaY, unfilteredPoint.getYoY());
      AlphaFilteredYoVariable z = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry, alphaZ, unfilteredPoint.getYoZ());
      
      AlphaFilteredYoFramePoint ret = new AlphaFilteredYoFramePoint(x, y, z, unfilteredPoint.getReferenceFrame());
      
      return ret;
   }
   
   public static AlphaFilteredYoFramePoint createAlphaFilteredYoFramePoint(String namePrefix, String nameSuffix, YoVariableRegistry registry,
         YoDouble alphaX, YoDouble alphaY, YoDouble alphaZ, YoFramePoint unfilteredPoint)
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
