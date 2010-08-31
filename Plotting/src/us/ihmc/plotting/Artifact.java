package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Graphics;
import java.io.Serializable;

public abstract class Artifact implements Plottable, Serializable
{
   /**
    * 
    */
   private static final long serialVersionUID = -463773605470590581L;
   protected String id;
   protected String type;
   protected int level;
   protected Color color = Color.blue;
   protected boolean showID = false;
   protected boolean isVisible = true;

   public Artifact(String id)
   {
      this.id = id;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public abstract void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor);

   public abstract void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor);

   public void setID(String id)
   {
      this.id = id;
   }

   public void setType(String type)
   {
      this.type = type;
   }

   public String getID()
   {
      return id;
   }

   public String getType()
   {
      return type;
   }

   public int getLevel()
   {
      return level;
   }

   public void setLevel(int level)
   {
      this.level = level;
   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   public void setShowID(boolean value)
   {
      showID = value;
   }

   public Color getColor()
   {
      return color;
   }

   public boolean isVisible()
   {
      return isVisible;
   }

   public void setVisible(boolean isVisible)
   {
      this.isVisible = isVisible;
   }

   public String toString()
   {
      return getID();
   }
}
