package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.HashMap;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class BoundsPainter implements PostProcessor
{
   private HashMap<Integer, int[]> boundingBoxes = new HashMap<Integer, int[]>();
   private float lineThickness = 1.0f;
   private int imageHeight = 480;

   public BoundsPainter(float lineThickness)
   {
      this.lineThickness = lineThickness;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public void setBoundingBoxes(HashMap<Integer, int[]> boundingBoxes)
   {
      this.boundingBoxes = boundingBoxes;
   }

   public void clearLines()
   {
      boundingBoxes.clear();
   }

   public void paint(Graphics graphics)
   {
      Color originalGraphicsColor = graphics.getColor();
      graphics.setColor(Color.red);
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();
      g2d.setStroke(new BasicStroke(lineThickness));

      for (int[] boundingBox : boundingBoxes.values())
      {
         int x = boundingBox[0];
         int y = boundingBox[1];
         int width = boundingBox[2] - boundingBox[0];
         int height = boundingBox[3] - boundingBox[1];
         graphics.drawRect(x, y, width, height);
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
