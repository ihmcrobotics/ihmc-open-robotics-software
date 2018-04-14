package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoGraphicLineSegment extends YoGraphicVector
{
   private final YoFramePoint3D start, end;
   private final YoFrameVector3D vector;

   public YoGraphicLineSegment(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, AppearanceDefinition appearance,
                               YoVariableRegistry registry)
   {
      this(namePrefix, new YoFramePoint3D(namePrefix, nameSuffix + "Start", referenceFrame, registry),
           new YoFramePoint3D(namePrefix, nameSuffix + "End", referenceFrame, registry), appearance);
   }

   public YoGraphicLineSegment(String name, YoFramePoint3D startPoint, YoFramePoint3D endPoint, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, 1.0, appearance);
   }

   public YoGraphicLineSegment(String name, YoFramePoint3D startPoint, YoFramePoint3D endPoint, double scale, AppearanceDefinition appearance)
   {
      this(name, startPoint, endPoint, scale, appearance, false);
   }

   public YoGraphicLineSegment(String name, YoFramePoint3D startPoint, YoFramePoint3D endPoint, double scale, AppearanceDefinition appearance,
                               boolean drawArrowhead)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), endPoint.getYoX(), endPoint.getYoY(), endPoint.getYoZ(), scale, appearance,
           drawArrowhead);

      if (!startPoint.getReferenceFrame().isWorldFrame() || !endPoint.getReferenceFrame().isWorldFrame())
      {
         System.err.println("Warning: Should be in a World Frame to create a YoGraphicLineSegment. startPoint = " + startPoint + ", endPoint = " + endPoint);
      }
   }

   public YoGraphicLineSegment(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z, double scaleFactor,
                               AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, true);
   }

   public YoGraphicLineSegment(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble endX, YoDouble endY, YoDouble endZ, double scaleFactor,
                               AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, baseX, baseY, baseZ, endX, endY, endZ, createDirectionVector(name, baseX.getYoVariableRegistry()), scaleFactor, appearance, drawArrowhead);
   }

   private static YoFrameVector3D createDirectionVector(String name, YoVariableRegistry registry)
   {
      YoFrameVector3D directionVector = new YoFrameVector3D(name, "Direction", ReferenceFrame.getWorldFrame(), registry);
      return directionVector;
   }

   private YoGraphicLineSegment(String name, YoDouble startX, YoDouble startY, YoDouble startZ, YoDouble endX, YoDouble endY, YoDouble endZ,
                                YoFrameVector3D yoFrameVector, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      super(name, startX, startY, startZ, yoFrameVector.getYoX(), yoFrameVector.getYoY(), yoFrameVector.getYoZ(), scaleFactor, appearance, drawArrowhead);

      vector = yoFrameVector;

      start = new YoFramePoint3D(startX, startY, startZ, ReferenceFrame.getWorldFrame());
      end = new YoFramePoint3D(endX, endY, endZ, ReferenceFrame.getWorldFrame());
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      if (vector == null)
      {
         return;
      }

      vector.sub(end, start);

      super.computeRotationTranslation(transform3D);
   }

   @Override
   public YoDouble[] getVariables()
   {
      return new YoDouble[] {start.getYoX(), start.getYoY(), start.getYoZ(), end.getYoX(), end.getYoY(), end.getYoZ()};
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {scaleFactor};
   }

   public void setStartAndEnd(FramePoint3DReadOnly startPoint, FramePoint3DReadOnly endPoint)
   {
      startPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      endPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setStartAndEnd((Point3DReadOnly) startPoint, (Point3DReadOnly) endPoint);
   }

   public void setStartAndEnd(Point3DReadOnly startPoint, Point3DReadOnly endPoint)
   {
      start.set(startPoint);
      end.set(endPoint);
      vector.sub(endPoint, startPoint);
   }

   public void setToNaN()
   {
      start.setToNaN();
      end.setToNaN();
      vector.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public YoGraphicLineSegment duplicate(YoVariableRegistry newRegistry)
   {
      return new YoGraphicLineSegment(getName(), start, end, scaleFactor, getAppearance(), getDrawArrowhead());
   }
}
