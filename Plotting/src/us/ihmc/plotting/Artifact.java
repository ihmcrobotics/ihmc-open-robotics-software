package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Graphics;
import java.io.Serializable;

public abstract class Artifact implements Plottable, Serializable
{
   private static final long serialVersionUID = -463773605470590581L;
   protected final String id;
   protected String type;
   protected int level;
   protected Color color = Color.blue;
   protected boolean isVisible = true;
   private boolean showID = false;
   private boolean drawHistory = false;
   private boolean recordHistory = false;
   

   public Artifact(String id)
   {
      this.id = id;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public abstract void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor);
   public abstract void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor);
   public abstract void takeHistorySnapshot();
   public abstract void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor);

   public void setType(String type)
   {
      this.type = type;
   }
   
   public void setShowID(boolean showID)
   {
      this.showID = showID;
   }
   
   public boolean getShowID()
   {
      return showID;
   }
   
   public void setDrawHistory(boolean drawHistory)
   {
      this.drawHistory = drawHistory;
   }
   
   public boolean getDrawHistory()
   {
      return drawHistory;
   }
   
   public void setRecordHistory(boolean recordHistory)
   {
      this.recordHistory = recordHistory;
   }
   
   public boolean getRecordHistory()
   {
      return recordHistory;
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

   public String getName() {
      return getID();
   }
   
   public String toString()
   {
      return getID();
   }
   

   
}
