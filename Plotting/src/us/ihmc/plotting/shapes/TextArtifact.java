package us.ihmc.plotting.shapes;

import java.awt.Font;
import java.awt.Graphics;
import java.io.PrintWriter;

import us.ihmc.plotting.Artifact;

public class TextArtifact extends Artifact
{
   /**
    *
    */
   private static final long serialVersionUID = 4004880709697051705L;
   private double x1;
   private double y1;
   private String text;
   private Font font = Font.getFont(Font.SANS_SERIF);
   private int xPixelOffset = 0;
   private int yPixelOffset = 0;

   public TextArtifact(String id, String text, double x1, double y1)
   {
      super(id);
      setLevel(1);
      this.text = text;
      this.x1 = x1;
      this.y1 = y1;
   }

   public void setPosition(double x1, double y1)
   {
      this.x1 = x1;
      this.y1 = y1;
   }

   public void setPixelOffset(int pixelOffset)
   {
      this.xPixelOffset = pixelOffset;
      this.yPixelOffset = pixelOffset;
   }

   public void setxPixelOffset(int xPixelOffset)
   {
      this.xPixelOffset = xPixelOffset;
   }

   public void setyPixelOffset(int yPixelOffset)
   {
      this.yPixelOffset = yPixelOffset;
   }

   public String getText()
   {
      return text;
   }

   public void setText(String text)
   {
      this.text = text;
   }

   public double getX()
   {
      return x1;
   }

   public double getY()
   {
      return y1;
   }

   public void setFontSize(int size)
   {
      font = new Font(Font.SANS_SERIF, Font.PLAIN, size);
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      int x1 = Xcenter + xPixelOffset +((int)Math.round(this.x1 * scaleFactor));
      int y1 = Ycenter - yPixelOffset - ((int)Math.round(this.y1 * scaleFactor));


      g.setColor(color);
      g.setFont(font);

      g.drawString(text, x1, y1);

   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x1 = Xcenter + ((int)Math.round(this.x1 * scaleFactor));
      int y1 = Ycenter - ((int)Math.round(this.y1 * scaleFactor));


      g.setColor(color);
      g.setFont(font);

      g.drawString(text, x1, y1);

   }

   public void save(PrintWriter printWriter)
   {
      printWriter.println(x1 + " " + y1 + " " + id);
   }



   public TextArtifact getCopy()
   {
      TextArtifact cirlceCopy = new TextArtifact(this.getID(), this.text, x1, y1);
      cirlceCopy.setColor(this.getColor());

      return cirlceCopy;
   }
   
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
   
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

}
