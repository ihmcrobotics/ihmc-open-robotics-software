package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class BoundsPainter implements PostProcessor
{
   private List<BoundingBox2D> boundingBoxes = new ArrayList<BoundingBox2D>();
   private float lineThickness = 1.0f;
   private int imageHeight = 544;

   public BoundsPainter(float lineThickness)
   {
      this.lineThickness = lineThickness;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public void setBoundingBoxes(List<BoundingBox2D> boundingBoxes)
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

      for (BoundingBox2D boundingBox : boundingBoxes)
      {
         int x = new Double(boundingBox.getMinPoint().getX()).intValue();
         int y = new Double(boundingBox.getMinPoint().getY()).intValue();
         int width = new Double(boundingBox.getMaxPoint().getX() - boundingBox.getMinPoint().getX()).intValue();
         int height = new Double(boundingBox.getMaxPoint().getY() - boundingBox.getMinPoint().getY()).intValue();
         graphics.drawRect(x, y, width, height);
         graphics.setColor(Color.cyan);
         graphics.fillOval(x, y, 5, 5);
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
