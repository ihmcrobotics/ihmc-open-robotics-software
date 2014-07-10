package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Image;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class ImagePanel extends JPanel
{
   private static final long serialVersionUID = -5134015392688757619L;

   private Image image;
   private JLabel picLabel;
   
   public ImagePanel(String filename)
   {
      try
      {
         image = ImageIO.read(ImagePanel.class.getClassLoader().getResource(filename));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      scaleImage(100, 100);
   }
   
   public ImagePanel(String filename, int width, int height)
   {
      this(filename);
      scaleImage(width, height);
   }
   
   public void scaleImage(int width, int height)
   {
      image = image.getScaledInstance(width, height, Image.SCALE_DEFAULT);
      picLabel = new JLabel(new ImageIcon(image));

      removeAll();
      add(picLabel);
      
      revalidate();
   }
   
   public static void main(String[] args)
   {
      JFrame frame = new JFrame();
      ImagePanel panel = new ImagePanel("ihmcRoboticsBlue.png", 300, 100);
      
      frame.add(panel);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.pack();
      frame.setVisible(true);
   }
}
