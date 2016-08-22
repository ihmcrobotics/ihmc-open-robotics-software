package us.ihmc.simulationconstructionset.plotting;

import java.awt.Color;

import us.ihmc.plotting.Drawing2DTools;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

public class DynamicGraphicYoCircleArtifact extends Artifact
{
   private final YoFramePoint2d center;
   private final DoubleYoVariable radius;
   private Boolean isFilledCircle = false;

   private static final int MIN_RADIUS = 5;
   private static final int MAX_RADIUS = 25;

   public DynamicGraphicYoCircleArtifact(String name, YoFramePoint2d center, DoubleYoVariable radius, Color color, Boolean isFilledCircle)
   {
      this(name, center, radius, color);
      this.isFilledCircle = isFilledCircle;
   }

   public DynamicGraphicYoCircleArtifact(String name, YoFramePoint2d center, DoubleYoVariable radius, Color color)
   {
      super(name);
      this.center = center;
      this.radius = radius;
      this.color = color;
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      double radius = Math.round(2.0 * this.radius.getDoubleValue() * scaleFactor);
      radius = MathTools.clipToMinMax(radius, MIN_RADIUS, MAX_RADIUS);
      draw(graphics, Xcenter, Ycenter, (int) radius);
   }

   @Override
   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      draw(graphics, center.getX(), center.getY(), Xcenter, Ycenter, scaleFactor);
   }

   private void draw(Graphics2DAdapter graphics, double xWorld, double yWorld, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = (int) (xWorld * scaleFactor) + Xcenter;
      int y = (int) (-yWorld * scaleFactor) + Ycenter;
      int radius = (int) (2.0 * this.radius.getDoubleValue() * scaleFactor);

      draw(graphics, x, y, radius);
   }

   private void draw(Graphics2DAdapter graphics, int x, int y, int radius)
   {
      if (isFilledCircle)
      {
         Drawing2DTools.drawFilledCircle(graphics, x, y, radius, color);
      }
      else
      {
         Drawing2DTools.drawEmptyCircle(graphics, x, y, radius, color);
      }
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}