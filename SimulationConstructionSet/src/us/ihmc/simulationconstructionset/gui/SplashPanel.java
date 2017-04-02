package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.MediaTracker;
import java.awt.Toolkit;
import java.net.URL;

import javax.swing.JPanel;
import javax.swing.JWindow;

public class SplashPanel extends JPanel
{
   private static final long serialVersionUID = -6476659432543699862L;
   private Image splashImage;
   private boolean splashImageLoaded = false;

   public SplashPanel()
   {
      super();
      setBackground(new Color(0, 255, 0, 0));
      URL imageURL = SplashPanel.class.getClassLoader().getResource("images/SplashScreen.png");

      Toolkit toolkit = Toolkit.getDefaultToolkit();
      splashImage = toolkit.getImage(imageURL);

      MediaTracker tracker = new MediaTracker(this);
      tracker.addImage(splashImage, 1);

      waitABitForMediaToLoad(tracker);
      
      if (tracker.checkAll())
         splashImageLoaded = true;
      else
         splashImageLoaded = false;
   }

   private void waitABitForMediaToLoad(MediaTracker tracker)
   {
      int numberOfWaitCycles = 0;
      while((numberOfWaitCycles < 10) && (!tracker.checkAll(true)))
      {
         try
         {
            //         tracker.waitForAll();
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
         numberOfWaitCycles++;
      }
   }
   
   public Dimension getImageSize()
   {
      return new Dimension(splashImage.getWidth(this), splashImage.getHeight(this));
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      g.drawImage(splashImage, 0, 0, this);
   }

   public boolean isSplashImageLoaded()
   {
      return splashImageLoaded;
   }

   public JWindow showSplashScreen()
   {
      // Wait until loaded:
      int numberOfAttempts = 0;
      while ((numberOfAttempts<10) && (!this.isSplashImageLoaded()))
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
         numberOfAttempts++;
      }

      JWindow splashWindow = new JWindow();
      splashWindow.setBackground(new Color(0, 255, 0, 0));

      Container splashContentPane = splashWindow.getContentPane();
      splashContentPane.add(this);

      Dimension imageSize = this.getImageSize();

      splashWindow.setSize(new Dimension(imageSize));

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

      splashWindow.setLocation((screenSize.width - imageSize.width) / 2, (screenSize.height - imageSize.height) / 2);
      splashWindow.validate();    // pack();
      splashWindow.setVisible(true);

      return splashWindow;
   }


}
