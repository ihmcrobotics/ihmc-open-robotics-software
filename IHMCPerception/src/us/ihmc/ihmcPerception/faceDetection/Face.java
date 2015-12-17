package us.ihmc.ihmcPerception.faceDetection;

import java.awt.Color;

import org.opencv.core.Rect;

public class Face
{
   private final long id;

   public Rect facialBorder;

//   public Color borderColor;
   public int rgb;
   
   public Face()
   {
      id = 0;
      rgb = Color.RED.getRGB();
   }
   
   public Face(long id, Rect facialBorder)
   {
      this.id = id;
      this.facialBorder = facialBorder;
      
      rgb = new Color((int) id).getRGB();
   }

   public void updateCoordinates(Rect coordinates)
   {
      this.facialBorder.set(new double[]{coordinates.x, coordinates.y, coordinates.width, coordinates.height});
   }

   public long getId()
   {
      return id;
   }

   //Have a getter just for cleanness of code. Don't take advantage that the field has to be public for packet issues
   public Rect getFacialBorder()
   {
      return facialBorder;
   }
   
   //Have a getter just for cleanness of code. Don't take advantage that the field has to be public for packet issues
   public Color getColor()
   {
      return new Color(rgb);
   }
}
