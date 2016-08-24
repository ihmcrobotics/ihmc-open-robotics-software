package us.ihmc.plotting.artifact;

import javax.swing.ImageIcon;

import us.ihmc.plotting.Graphics2DAdapter;

public class CenteredImageIconArtifact extends Artifact
{
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
   public void draw(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         int xPosition = Xcenter - imageIcon.getIconWidth()/2;
         int yPosition = Ycenter - imageIcon.getIconHeight()/2;
         
         graphics2d.drawImage(imageIcon.getImage(), xPosition, yPosition, null);
      }
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
}
