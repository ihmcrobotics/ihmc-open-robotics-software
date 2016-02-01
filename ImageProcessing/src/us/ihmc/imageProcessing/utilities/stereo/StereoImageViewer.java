package us.ihmc.imageProcessing.utilities.stereo;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;

import javax.swing.JPanel;

/**
 * Last updated by: mjohnson
 * On: 5/11/11 4:57 PM
 */
@SuppressWarnings("serial")
public class StereoImageViewer extends JPanel implements StereoVideoListener
{
   protected Image _leftImage;
   protected Image _rightImage;


   public StereoImageViewer()
   {
      Dimension dimension = new Dimension(640, 240);
      setPreferredSize(dimension);
      repaint();
   }

   public StereoImageViewer(Dimension preferredSize)
   {
      setPreferredSize(preferredSize);
      repaint();
   }



   public void updateImage(BufferedImage leftImage, BufferedImage rightImage)
   {
      _leftImage = leftImage;
      _rightImage = rightImage;
      repaint();
   }

   public void paint(Graphics graphics)
   {
      update(graphics);
   }

   public void update(Graphics g)
   {
      if ((_leftImage != null) && (_rightImage != null))
      {
         g.drawImage(_leftImage, 0, 0, _leftImage.getWidth(null), _leftImage.getHeight(null), this);
         g.drawImage(_rightImage, _leftImage.getWidth(null), 0, _rightImage.getWidth(null), _rightImage.getHeight(null), this);
        
      }

      else
      {
         Color c = g.getColor();
         g.setColor(Color.white);
         g.fillRect(0, 0, this.getWidth(), this.getHeight());
         g.setColor(c);
      }
   }
}
