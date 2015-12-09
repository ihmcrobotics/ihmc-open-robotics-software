package us.ihmc.ihmcPerception.faceDetection;

import org.opencv.core.Rect;

import java.awt.*;

public class Face
{
   private final long id;
   public Rect facialBorder;
   public Color borderColor;

   public Face(long id, Rect facialBorder)
   {
      this.id = id;
      this.facialBorder = facialBorder;
      borderColor = new Color((int)id);
   }

   public void updateCoordinates(Rect coordinates)
   {
      this.facialBorder.set(new double[]{coordinates.x, coordinates.y, coordinates.width, coordinates.height});
   }
}
