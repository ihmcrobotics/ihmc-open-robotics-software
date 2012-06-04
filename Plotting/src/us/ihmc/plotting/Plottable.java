package us.ihmc.plotting;

import java.awt.Graphics;

public interface Plottable
{
   public static int X_Y = 0;
   public static int X_Z = 1;
   public static int Y_Z = 2;

   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor);

   public String getID();

   public String getType();

   public int getLevel();
}
