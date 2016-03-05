package us.ihmc.imageProcessing.utilities;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

/**
 * Last updated by: mjohnson
 * On: 5/11/11 4:57 PM
 */
@SuppressWarnings("serial")
public class ImageViewer extends JPanel implements VideoListener
{
   protected Image _image;

   public ImageViewer()
   {
      Dimension dimension = new Dimension(320, 240);
      setPreferredSize(dimension);
      repaint();
   }

   public ImageViewer(Dimension preferredSize)
   {

      setPreferredSize(preferredSize);
      repaint();
   }

   public ImageViewer(String fileName)
   {
      Dimension dimension = new Dimension(320, 240);
      setPreferredSize(dimension);
      Toolkit toolkit = Toolkit.getDefaultToolkit();
      _image = toolkit.getImage(fileName);    // .getScaledInstance(_width , _height - 20, 1);
      repaint();
   }

   public Image getImageFromJAR(String fileName)
   {
      try
      {
         if (fileName == null)
         {
            return null;
         }
         Image image = null;
         Toolkit toolkit = Toolkit.getDefaultToolkit();
         image = toolkit.getImage(getClass().getResource(fileName));

         return image;
      }
      catch (Exception exc)
      {
         JOptionPane.showMessageDialog(new JFrame(), "Exception loading the image", "Error", JOptionPane.ERROR_MESSAGE);

         return null;
      }
   }

   public void updateImage(String fileName)
   {
      int _width = 320;
      _image = Toolkit.getDefaultToolkit().getImage(fileName).getScaledInstance(_width, -1, 1);
      repaint();
   }

   public void updateImage(BufferedImage image)
   {
      _image = image;
      repaint();
   }

   public void updateImage(Image image)
   {
      _image = image;
      repaint();
   }

   public void paint(Graphics graphics)
   {
      update(graphics);
   }

   public void update(Graphics g)
   {
      if (_image != null)
      {
         g.drawImage(_image, 0, 0, _image.getWidth(null), _image.getHeight(null), this);
      }
      else
      {
         Color c = g.getColor();
         g.setColor(Color.white);
         g.fillRect(0, 0, this.getWidth(), this.getHeight());
         g.setColor(c);
      }
   }

   public static void main(String[] args)
   {
      try
      {
         String testImage = "./media/5.png";
         JFrame f = new JFrame("Image Viewer");
         f.addWindowListener(new WindowAdapter()
         {
            public void windowClosing(WindowEvent e)
            {
               System.exit(0);
            }
         });

         ImageViewer imageViewer = new ImageViewer(testImage);
         f.getContentPane().add(imageViewer, BorderLayout.CENTER);
         f.pack();
         f.setVisible(true);

         // countdown
         for (int i = 4; i > 1; i--)
         {
            Thread.sleep(1000);
            String fileName = "./media/" + (i) + ".png";
            imageViewer.updateImage(fileName);
         }

         Thread.sleep(1000);
         String fileName = "./media/NoImage.png";
         imageViewer.updateImage(fileName);
         Thread.sleep(1000);

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
