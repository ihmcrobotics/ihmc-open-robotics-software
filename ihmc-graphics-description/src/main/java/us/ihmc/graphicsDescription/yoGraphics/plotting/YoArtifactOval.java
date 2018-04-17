package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

public class YoArtifactOval extends YoArtifact
{
   private static final int LEGEND_DIAMETER = 10;
   
   private final YoFramePoint2D center;
   private final YoFrameVector2D radii;
   
   private final Point2D tempCenter = new Point2D();
   private final Vector2D tempRadii = new Vector2D();

   public YoArtifactOval(String name, YoDouble centerX, YoDouble centerY, YoDouble radius, Color color)
   {
      this(name, centerX, centerY, radius, radius, color);
   }

   public YoArtifactOval(String name, YoFramePoint3D center, YoDouble radius, Color color)
   {
      this(name, center.getYoX(), center.getYoY(), radius, radius, color);
   }
   
   private YoArtifactOval(String name, YoDouble centerX, YoDouble centerY, YoDouble radiusX, YoDouble radiusY, Color color)
   {
      this(name, new YoFramePoint2D(centerX, centerY, ReferenceFrame.getWorldFrame()),
                 new YoFrameVector2D(radiusX, radiusY, ReferenceFrame.getWorldFrame()), color);
   }
   
   public YoArtifactOval(String name, YoFramePoint2D center, YoFrameVector2D radii, Color color)
   {
      super(name, new double[0], color,
            center.getYoX(), center.getYoY(), radii.getYoX(), radii.getYoY());
      this.center = center;
      this.radii = radii;
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
   {
      graphics.setColor(color);
      tempCenter.set(origin);
      tempRadii.set(LEGEND_DIAMETER / 2.0, LEGEND_DIAMETER / 2.0);
      graphics.drawOval(graphics.getScreenFrame(), tempCenter, tempRadii);
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      tempCenter.set(center);
      tempRadii.set(radii);
      graphics.setColor(color);
      graphics.drawOval(tempCenter, tempRadii);
   }

   @Override
   public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
   {
      tempCenter.set(entry[0], entry[1]);
      tempRadii.set(entry[2], entry[3]);

      graphics.setColor(color);
      graphics.drawOval(tempCenter, tempRadii);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.CIRCLE_ARTIFACT;
   }
}
