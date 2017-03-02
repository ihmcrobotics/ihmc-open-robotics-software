package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.Color;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactOval extends YoArtifact
{
   private static final int LEGEND_DIAMETER = 10;
   
   private final YoFramePoint2d center;
   private final YoFrameVector2d radii;
   
   private final Point2D tempCenter = new Point2D();
   private final Vector2D tempRadii = new Vector2D();

   public YoArtifactOval(String name, DoubleYoVariable centerX, DoubleYoVariable centerY, DoubleYoVariable radius, Color color)
   {
      this(name, centerX, centerY, radius, radius, color);
   }

   public YoArtifactOval(String name, YoFramePoint center, DoubleYoVariable radius, Color color)
   {
      this(name, center.getYoX(), center.getYoY(), radius, radius, color);
   }
   
   private YoArtifactOval(String name, DoubleYoVariable centerX, DoubleYoVariable centerY, DoubleYoVariable radiusX, DoubleYoVariable radiusY, Color color)
   {
      this(name, new YoFramePoint2d(centerX, centerY, ReferenceFrame.getWorldFrame()),
                 new YoFrameVector2d(radiusX, radiusY, ReferenceFrame.getWorldFrame()), color);
   }
   
   public YoArtifactOval(String name, YoFramePoint2d center, YoFrameVector2d radii, Color color)
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
      center.get(tempCenter);
      radii.get(tempRadii);
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
