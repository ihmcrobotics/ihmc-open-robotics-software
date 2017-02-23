package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class CirclePainter implements PostProcessor
{
   private ArrayList<Vector3D> circles = new ArrayList<Vector3D>();
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

   public void setCircles(ArrayList<Vector3D> circles)
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

      for (Vector3D circle : circles)
      {
         graphics.drawOval(new Double(circle.getX()).intValue(), new Double(circle.getY()).intValue(), new Double(circle.getZ()).intValue(), new Double(circle.getZ()).intValue());
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
