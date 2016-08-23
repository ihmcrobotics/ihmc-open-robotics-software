package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Drawing2DTools;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

public class YoArtifactCircle extends Artifact implements RemoteYoGraphic
{
   private final ArrayList<double[]> historicalData = new ArrayList<double[]>();

   private DoubleYoVariable x;
   private DoubleYoVariable y;
   private DoubleYoVariable radius;

   public YoArtifactCircle(String name, DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable radius, Color color)
   {
      super(name);
      this.x = x;
      this.y = y;
      this.radius = radius;

      this.color = color;
   }

   public YoArtifactCircle(String name, YoFramePoint circleCenter, DoubleYoVariable radius, Color color)
   {
      super(name);
      this.x = circleCenter.getYoX();
      this.y = circleCenter.getYoY();
      this.radius = radius;
      this.color = color;
   }

   @Override
   public void takeHistorySnapshot()
   {
      if (getRecordHistory())
      {
         synchronized (historicalData)
         {
            historicalData.add(new double[] {x.getDoubleValue(), y.getDoubleValue(), radius.getDoubleValue()});
         }
      }
   }

   private final Point3d position = new Point3d();
   private static final int MIN_RADIUS = 5;
   private static final int MAX_RADIUS = 25;

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      double adjustedRadius = Math.round(4.0 * radius.getDoubleValue() * scaleFactor);
      adjustedRadius = MathTools.clipToMinMax(adjustedRadius, MIN_RADIUS, MAX_RADIUS);
      draw(graphics, Xcenter, Ycenter, (int) adjustedRadius);
   }

   @Override
   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         getPosition(position);
         draw(graphics, position.x, position.y, Xcenter, Ycenter, radius.getDoubleValue(), scaleFactor);
      }
   }

   public void getPosition(Point3d point3d)
   {
      point3d.set(x.getDoubleValue(), y.getDoubleValue(), 0.0);
   }

   private void draw(Graphics2DAdapter graphics2d, double xWorld, double yWorld, int Xcenter, int Ycenter, double radius, double scaleFactor)
   {
      if (Double.isNaN(xWorld) || Double.isNaN(yWorld))
         return;

      int x = (int) (xWorld * scaleFactor) + Xcenter;
      int y = (int) (-yWorld * scaleFactor) + Ycenter;
      int adjustedRadius = (int) (2.0 * radius * scaleFactor);

      draw(graphics2d, x, y, adjustedRadius);
   }

   private void draw(Graphics2DAdapter graphics, int x, int y, int radius)
   {
      Drawing2DTools.drawEmptyCircle(graphics, x, y, radius, color);
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      synchronized (historicalData)
      {
         for (double[] data : historicalData)
         {
            double xWorld = data[0];
            double yWorld = data[1];
            double radius = data[2];

            draw(graphics, xWorld, yWorld, Xcenter, Ycenter, radius, scaleFactor);
         }
      }
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.CIRCLE_ARTIFACT;
   }

   @Override
   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] {x, y, radius};
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {};
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return new YoAppearanceRGBColor(color, 0.0);
   }
}
