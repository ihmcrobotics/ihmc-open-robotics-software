package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class LinePainter implements PostProcessor
{
   private ArrayList<Line2D> lines = new ArrayList<Line2D>();
   private float lineThickness = 1.0f;
   private int imageHeight = 480;

   public LinePainter(float lineThickness)
   {
      this.lineThickness = lineThickness;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public void setLines(ArrayList<Line2D> lines)
   {
      this.lines = lines;
   }

   public void clearLines()
   {
      lines.clear();
   }

   public void paint(Graphics graphics)
   {
      Color originalGraphicsColor = graphics.getColor();
      graphics.setColor(Color.red);
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();
      g2d.setStroke(new BasicStroke(lineThickness));
      Line2D xMin = new Line2D(new Point2D(0, 0), new Vector2D(1.0, 0.0));
      Line2D xMax = new Line2D(new Point2D(0, imageHeight), new Vector2D(1.0, 0.0));
      for (Line2D line : lines)
      {
         Point2DBasics p1 = line.intersectionWith(xMin);
         Point2DBasics p2 = line.intersectionWith(xMax);
         graphics.drawLine((int) p1.getX(), (int)p1.getY(), (int)p2.getX(), (int) p2.getY());
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
