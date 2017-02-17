package us.ihmc.robotics.dataStructures;

import java.awt.Color;
import java.util.StringTokenizer;

public class MutableColor
{
   public float x, y, z;

   public MutableColor()
   {
   }

   public MutableColor(MutableColor color)
   {
      set(color);
   }

   public MutableColor(Color color)
   {
      set(color);
   }

   public MutableColor(float x, float y, float z)
   {
      set(x, y, z);
   }

   public MutableColor(float[] color)
   {
      set(color[0], color[1], color[2]);
   }

   public void set(MutableColor other)
   {
      set(other.x, other.y, other.z);
   }

   public void set(float x, float y, float z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   public final void set(Color color)
   {
      x = (float) color.getRed() / 255.0f;
      y = (float) color.getGreen() / 255.0f;
      z = (float) color.getBlue() / 255.0f;
   }

   public final Color get()
   {
      int r = Math.round(x * 255.0f);
      int g = Math.round(y * 255.0f);
      int b = Math.round(z * 255.0f);

      return new Color(r, g, b);
   }

   public float getX()
   {
      return x;
   }

   public float getY()
   {
      return y;
   }

   public float getZ()
   {
      return z;
   }

   public void setX(double x)
   {
      this.x = (float) x;
   }

   public void setY(double y)
   {
      this.y = (float) y;

   }

   public void setZ(double z)
   {
      this.z = (float) z;
   }

   public static MutableColor parseColor3f(String color)
   {
      try
      {
         StringTokenizer s = new StringTokenizer(color, ",");

         return new MutableColor(new float[] {Float.parseFloat(s.nextToken()), Float.parseFloat(s.nextToken()), Float.parseFloat(s.nextToken())});
      }
      catch (Exception e)
      {
         return null;
      }
   }
}
