package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import javax.vecmath.Point2d;

import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.math.frames.YoFrameLine2d;

public class YoArtifactLine2d extends YoArtifact
{
   private static final BasicStroke STROKE = new BasicStroke(2);
   
   private final YoFrameLine2d yoFrameLine2d;
   
   private final Line2d tempLine = new Line2d();
   
   public YoArtifactLine2d(String name, YoFrameLine2d yoFrameLine2d, Color color)
   {
      super(name, new double[0], color,
            yoFrameLine2d.getYoPointX(), yoFrameLine2d.getYoPointY(), yoFrameLine2d.getYoVectorX(), yoFrameLine2d.getYoVectorY());
      this.yoFrameLine2d = yoFrameLine2d;
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      yoFrameLine2d.getFrameLine2d().get(tempLine);
      graphics.drawLine(tempLine);
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      graphics.drawLineSegment(graphics.getScreenFrame(), -20.0 + origin.getX(), -5.0 + origin.getY(), 20.0 + origin.getX(), 5.0 + origin.getY());
   }

   @Override
   public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      tempLine.set(entry[0], entry[1], entry[2], entry[3]);
      graphics.drawLine(tempLine);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.LINE_ARTIFACT;
   }
}
