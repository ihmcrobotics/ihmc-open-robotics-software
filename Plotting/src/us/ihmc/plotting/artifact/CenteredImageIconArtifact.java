package us.ihmc.plotting.artifact;

import java.awt.Graphics;

import javax.swing.ImageIcon;

public class CenteredImageIconArtifact extends Artifact
{
   private static final long serialVersionUID = 8946412004186672487L;
   private ImageIcon imageIcon;
   
   public CenteredImageIconArtifact(String filepath)
   {
      super(filepath);
      imageIcon = new ImageIcon(filepath);
   }

   public int getHeight()
   {
      return imageIcon.getIconHeight();
   }

   public int getWidth()
   {
      return imageIcon.getIconWidth();
   }

   @Override
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         int xPosition = Xcenter - imageIcon.getIconWidth()/2;
         int yPosition = Ycenter - imageIcon.getIconHeight()/2;
         
         g.drawImage(imageIcon.getImage(), xPosition, yPosition, null);
      }
   }

   @Override
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
}
