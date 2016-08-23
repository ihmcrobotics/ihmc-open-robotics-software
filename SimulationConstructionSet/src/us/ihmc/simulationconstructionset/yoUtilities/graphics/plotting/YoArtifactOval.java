package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;

public class YoArtifactOval extends YoArtifact
{
   private static final int LEGEND_DIAMETER = 10;
   
   private DoubleYoVariable centerX;
   private DoubleYoVariable centerY;
   private DoubleYoVariable radiusX;
   private DoubleYoVariable radiusY;
   
   private final ArrayList<double[]> historicalData = new ArrayList<double[]>();

   private final Point2d center = new Point2d();
   private final Vector2d radii = new Vector2d();

   public YoArtifactOval(String name, DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable radius, Color color)
   {
      super(name);
      this.centerX = x;
      this.centerY = y;
      this.radiusX = radius;
      this.radiusY = radius;
      this.color = color;
   }

   public YoArtifactOval(String name, YoFramePoint center, DoubleYoVariable radius, Color color)
   {
      super(name);
      this.centerX = center.getYoX();
      this.centerY = center.getYoY();
      this.radiusX = radius;
      this.radiusY = radius;
      this.color = color;
   }
   
   public YoArtifactOval(String name, YoFramePoint2d center, YoFrameVector2d radii, Color color)
   {
      super(name);
      this.centerX = center.getYoX();
      this.centerY = center.getYoY();
      this.radiusX = radii.getYoX();
      this.radiusY = radii.getYoY();
      this.color = color;
   }

   @Override
   public void takeHistorySnapshot()
   {
      if (getRecordHistory())
      {
         synchronized (historicalData)
         {
            update();
            historicalData.add(new double[] {center.getX(), center.getY(), radii.getX(), radii.getY()});
         }
      }
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY, double scaleFactor)
   {
      update();
      graphics.setColor(color);
      graphics.drawOval(centerX, centerY, LEGEND_DIAMETER, LEGEND_DIAMETER);
   }

   @Override
   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         update();
         graphics.setColor(color);
         graphics.drawOval(center, radii);
      }
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      synchronized (historicalData)
      {
         for (double[] data : historicalData)
         {
            center.set(data[0], data[1]);
            radii.set(data[2], data[3]);

            graphics.setColor(color);
            graphics.drawOval(center, radii);
         }
      }
   }

   private void update()
   {
      center.set(centerX.getDoubleValue(), centerY.getDoubleValue());
      radii.set(radiusX.getDoubleValue(), radiusY.getDoubleValue());
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.CIRCLE_ARTIFACT;
   }

   @Override
   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] {centerX, centerY, radiusX, radiusY};
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
