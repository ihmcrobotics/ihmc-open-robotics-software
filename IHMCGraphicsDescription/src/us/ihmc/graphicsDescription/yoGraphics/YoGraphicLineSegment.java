package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoGraphicLineSegment extends YoGraphicVector
{
   private final DoubleYoVariable startX, startY, startZ, endX, endY, endZ;
   private final DoubleYoVariable vectorX, vectorY, vectorZ;

   public YoGraphicLineSegment(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, AppearanceDefinition appearance, YoVariableRegistry registry)
   {
      this(namePrefix, new YoFramePoint(namePrefix, nameSuffix + "Start", referenceFrame, registry), new YoFramePoint(namePrefix, nameSuffix + "End", referenceFrame, registry), appearance);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, 1.0, appearance);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, double scale, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, scale, appearance, false);
   }

   public YoGraphicLineSegment(String name, YoFramePoint startPoint, YoFramePoint endPoint, double scale, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), endPoint.getYoX(), endPoint.getYoY(), endPoint.getYoZ(), scale, appearance, drawArrowhead);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!endPoint.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a YoGraphicLineSegment. startPoint = " + startPoint + ", endPoint = " + endPoint);
      }
   }

   public YoGraphicLineSegment(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z, double scaleFactor,
         AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, true);
   }

   public YoGraphicLineSegment(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable endX, DoubleYoVariable endY, DoubleYoVariable endZ, double scaleFactor,
         AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, baseX, baseY, baseZ, endX, endY, endZ, createDirectionVector(name, baseX.getYoVariableRegistry()), scaleFactor, appearance, drawArrowhead);
   }

   private static YoFrameVector createDirectionVector(String name, YoVariableRegistry registry)
   {
      YoFrameVector directionVector = new YoFrameVector(name, "Direction", ReferenceFrame.getWorldFrame(), registry);
      return directionVector;
   }

   private YoGraphicLineSegment(String name, DoubleYoVariable startX, DoubleYoVariable startY, DoubleYoVariable startZ, DoubleYoVariable endX, DoubleYoVariable endY, DoubleYoVariable endZ, YoFrameVector yoFrameVector,
         double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      super(name, startX, startY, startZ, yoFrameVector.getYoX(), yoFrameVector.getYoY(), yoFrameVector.getYoZ(), scaleFactor, appearance, drawArrowhead);

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

   protected void computeRotationTranslation(AffineTransform transform3D)
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

   public void setStartAndEnd(Point3D startPoint, Point3D endPoint)
   {
      this.startX.set(startPoint.getX());
      this.startY.set(startPoint.getY());
      this.startZ.set(startPoint.getZ());

      this.endX.set(endPoint.getX());
      this.endY.set(endPoint.getY());
      this.endZ.set(endPoint.getZ());

      this.vectorX.set(endPoint.getX() - startPoint.getX());
      this.vectorY.set(endPoint.getY() - startPoint.getY());
      this.vectorZ.set(endPoint.getZ() - startPoint.getZ());
   }

   public void setToNaN()
   {
      startX.setToNaN();
      startY.setToNaN();
      startZ.setToNaN();

      endX.setToNaN();
      endY.setToNaN();
      endZ.setToNaN();

      vectorX.setToNaN();
      vectorY.setToNaN();
      vectorZ.setToNaN();
   }

}
