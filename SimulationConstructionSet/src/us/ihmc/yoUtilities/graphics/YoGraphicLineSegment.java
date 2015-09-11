package us.ihmc.yoUtilities.graphics;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class YoGraphicLineSegment extends YoGraphicVector
{
   private final DoubleYoVariable startX, startY, startZ, endX, endY, endZ;
   private final DoubleYoVariable vectorX, vectorY, vectorZ;

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, 1.0, appearance);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, double scale, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, scale, appearance, true);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, double scale, AppearanceDefinition appearance,
         boolean drawArrowhead)
   {
      this(name, startPoint, endPoint, scale, appearance, drawArrowhead, -1.0);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, double scale, AppearanceDefinition appearance,
         boolean drawArrowhead, double lineThicknessRatio)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), endPoint.getYoX(), endPoint.getYoY(), endPoint.getYoZ(), scale, appearance,
            drawArrowhead, lineThicknessRatio);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!endPoint.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a DynamicGraphicLineSegment. startPoint = " + startPoint + ", endPoint = "
               + endPoint);
      }
   }

   public YoGraphicLineSegment(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x,
         DoubleYoVariable y, DoubleYoVariable z, double scaleFactor, AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, true);
   }

   public YoGraphicLineSegment(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable endX,
         DoubleYoVariable endY, DoubleYoVariable endZ, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, baseX, baseY, baseZ, endX, endY, endZ, scaleFactor, appearance, drawArrowhead, -1.0);
   }

   public YoGraphicLineSegment(String name, DoubleYoVariable startX, DoubleYoVariable startY, DoubleYoVariable startZ, DoubleYoVariable endX,
         DoubleYoVariable endY, DoubleYoVariable endZ, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead, double lineThicknessRatio)
   {
      this(name, startX, startY, startZ, endX, endY, endZ, tempYoFrameVector(), scaleFactor, appearance, drawArrowhead, lineThicknessRatio);
   }

   private static YoFrameVector tempYoFrameVector()
   {
      YoVariable.warnAboutNullRegistries = false;
      YoFrameVector temp = new YoFrameVector("temp", "", ReferenceFrame.getWorldFrame(), null);
      YoVariable.warnAboutNullRegistries = true;

      return temp;
   }

   private YoGraphicLineSegment(String name, DoubleYoVariable startX, DoubleYoVariable startY, DoubleYoVariable startZ, DoubleYoVariable endX,
         DoubleYoVariable endY, DoubleYoVariable endZ, YoFrameVector yoFrameVector, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead,
         double lineThicknessRatio)
   {
      super(name, startX, startY, startZ, yoFrameVector.getYoX(), yoFrameVector.getYoY(), yoFrameVector.getYoZ(), scaleFactor, appearance, drawArrowhead,
            lineThicknessRatio);

      this.vectorX = yoFrameVector.getYoX();
      this.vectorY = yoFrameVector.getYoY();
      this.vectorZ = yoFrameVector.getYoZ();

      this.startX = startX;
      this.startY = startY;
      this.startZ = startZ;

      this.endX = endX;
      this.endY = endY;
      this.endZ = endZ;

      //    computeRotationTranslation();
   }

   protected void computeRotationTranslation(Transform3d transform3D)
   {
      if (vectorX == null)
      {
         return;
      }

      vectorX.set(endX.getDoubleValue() - startX.getDoubleValue());
      vectorY.set(endY.getDoubleValue() - startY.getDoubleValue());
      vectorZ.set(endZ.getDoubleValue() - startZ.getDoubleValue());

      super.computeRotationTranslation(transform3D);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.LINE_SEGMENT_DGO;
   }

   @Override
   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] { startX, startY, startZ, endX, endY, endZ };
   }

   @Override
   public double[] getConstants()
   {
      return new double[] { scaleFactor };
   }
}
