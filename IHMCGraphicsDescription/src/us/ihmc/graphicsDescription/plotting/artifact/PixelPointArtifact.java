package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Color;

import javax.vecmath.Point2d;

import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;

/**
 * To render pixel-radius points that auto-scale with the GUI.
 * 
 * @author Duncan Calvert (dcalvert@ihmc.us)
 */
public class PixelPointArtifact extends Artifact
{
   private final Point2d point;
   private BasicStroke basicStroke;
   
   public PixelPointArtifact(String id, Point2d point)
   {
      this(id, point, Color.BLACK, 1.0f);
   }
   
   public PixelPointArtifact(String id, Point2d point, Color color, float width)
   {
      super(id);
      
      this.point = point;
      basicStroke = new BasicStroke(width);
      setColor(color);
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(getColor());
      graphics.setStroke(basicStroke);
      graphics.drawPoint(point);
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics)
   {
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
   {
   }

   @Override
   public void takeHistorySnapshot()
   {
   }
}
