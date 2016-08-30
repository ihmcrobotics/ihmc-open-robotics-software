package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.plotting.Graphics2DAdapter;
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
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      graphics.drawLineSegment(-20 + centerX, -5 + centerY, 20 + centerX, 5 + centerY);
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
