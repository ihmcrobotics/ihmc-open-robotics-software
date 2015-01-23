package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class CirclePainter implements PostProcessor
{
   private ArrayList<Vector3d> circles = new ArrayList<Vector3d>();
   private float lineThickness = 1.0f;
   private int imageHeight = 480;

   public CirclePainter(float lineThickness)
   {
      this.lineThickness = lineThickness;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public void setCircles(ArrayList<Vector3d> circles)
   {
      this.circles = circles;
   }

   public void clearLines()
   {
      circles.clear();
   }

   public void paint(Graphics graphics)
   {
      Color originalGraphicsColor = graphics.getColor();
      graphics.setColor(Color.red);
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();
      g2d.setStroke(new BasicStroke(lineThickness));

      for (Vector3d circle : circles)
      {
         graphics.drawOval(new Double(circle.x).intValue(), new Double(circle.y).intValue(), new Double(circle.z).intValue(), new Double(circle.z).intValue());
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
