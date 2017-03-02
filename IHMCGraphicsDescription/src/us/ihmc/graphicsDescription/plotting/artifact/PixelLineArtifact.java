package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;

/**
 * To render pixel-radius lines that auto-scale with the GUI.
 * 
 * @author Duncan Calvert (dcalvert@ihmc.us)
 */
public class PixelLineArtifact extends Artifact
{
   private Point2D pointOne;
   private Point2D pointTwo;
   private BasicStroke basicStroke;
   
   public PixelLineArtifact(String id, Point2D pointOne, Point2D pointTwo)
   {
      this(id, pointOne, pointTwo, Color.BLACK, 1.0f);
   }
   
   public PixelLineArtifact(String id, Point2D pointOne, Point2D pointTwo, Color color, float width)
   {
      super(id);
      
      this.pointOne = pointOne;
      this.pointTwo = pointTwo;
      basicStroke = new BasicStroke(width);
      setColor(color);
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(getColor());
      graphics.setStroke(basicStroke);
      graphics.drawLineSegment(pointOne.getX(), pointOne.getY(), pointTwo.getX(), pointTwo.getY());
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics)
   {
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
   {
   }

   @Override
   public void takeHistorySnapshot()
   {
   }

   public void setPointOne(Point2D pointOne)
   {
      this.pointOne = pointOne;
   }

   public void setPointTwo(Point2D pointTwo)
   {
      this.pointTwo = pointTwo;
   }
}
