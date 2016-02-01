package us.ihmc.imageProcessing.utilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.BoundingBox2d;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class BoundsPainter implements PostProcessor
{
   private List<BoundingBox2d> boundingBoxes = new ArrayList<BoundingBox2d>();
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

   public void setBoundingBoxes(List<BoundingBox2d> boundingBoxes)
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

      for (BoundingBox2d boundingBox : boundingBoxes)
      {
         int x = new Double(boundingBox.getMinPoint().x).intValue();
         int y = new Double(boundingBox.getMinPoint().y).intValue();
         int width = new Double(boundingBox.getMaxPoint().x - boundingBox.getMinPoint().x).intValue();
         int height = new Double(boundingBox.getMaxPoint().y - boundingBox.getMinPoint().y).intValue();
         graphics.drawRect(x, y, width, height);
         graphics.setColor(Color.cyan);
         graphics.fillOval(x, y, 5, 5);
      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);

   }
}
