package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.net.URISyntaxException;

import javax.imageio.ImageIO;
import javax.swing.*;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AboutDialogGenerator implements AboutDialogConstructor
{
   private JFrame parentJFrame;
   
   private final String aboutLink = "https://ihmcrobotics.github.io/";
   private URI aboutURI = null;

   public AboutDialogGenerator(JFrame parentJFrame)
   {
      this.parentJFrame = parentJFrame;
   }

   @Override
   public void constructDialog()
   {
      try
      {
         aboutURI = new URI(aboutLink);
      } catch (URISyntaxException e) {
         System.err.println("Warning: About dialog URI is malformed.");
      }
      
      String scsVersionNumber = SimulationConstructionSet.getVersion();
      
      JDialog aboutDialog = new JDialog(parentJFrame, "Simulation Construction Set Version: " + scsVersionNumber, false);
      
      aboutDialog.getContentPane().setLayout(new BoxLayout(aboutDialog.getContentPane(), BoxLayout.Y_AXIS));
      
      InputStream imageStream = getClass().getClassLoader().getResourceAsStream("Banner-v1-Transparent-Inverted.png");
      if (imageStream == null)
      {
         System.err.println("Warning: About dialog image cannot be loaded.");
      } else {
         try
         {
            BufferedImage bufferedImage = ImageIO.read(imageStream);
            int desiredWidth = 450;
            int desiredHeight = 300;
            
            int currentWidth = bufferedImage.getWidth();
            int currentHeight = bufferedImage.getHeight();
            
            if (currentWidth > desiredWidth)
            {
               if (currentHeight * ((double)desiredWidth / currentWidth) > desiredHeight) {
                  desiredWidth = (int)(currentWidth * ((double)desiredHeight / currentHeight));
               } else {
                  desiredHeight = (int)(currentHeight * ((double)desiredWidth / currentWidth));
               }
            } else if (currentWidth < desiredWidth) {
               if (currentHeight * ((double)desiredWidth / currentWidth) < desiredHeight) {
                  desiredWidth = (int)(currentWidth * ((double)desiredHeight / currentHeight));
               } else {
                  desiredHeight = (int)(currentWidth * ((double)desiredHeight / currentHeight));
               }
            } else if (currentHeight != 300) {
               desiredWidth = (int)(currentWidth * ((double)desiredHeight / currentHeight));
            }
            
            Image finalImage = bufferedImage.getScaledInstance(desiredWidth, desiredHeight, Image.SCALE_DEFAULT);
            ImageIcon imageIcon = new ImageIcon(finalImage);
            JLabel imageLabel = new JLabel(imageIcon);
            imageLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
            
            aboutDialog.getContentPane().add(imageLabel);
         } catch (IOException e) {
            System.err.println("Warning: About dialog image not properly added.");
         }
      }

      JLabel version = new JLabel("Version "+scsVersionNumber);
      version.setFont(new Font("DTL Nobel", Font.PLAIN, 32));
      version.setAlignmentX(Component.CENTER_ALIGNMENT);
      aboutDialog.getContentPane().add(version);

      JButton github = new JButton();
      github.setText("<HTML><U>" + aboutLink + "</U></HTML>");
      github.setForeground(Color.BLUE);
      github.setFont(new Font("DTL Nobel", Font.PLAIN, 30));
      github.setBorderPainted(false);
      github.setContentAreaFilled(false);
      github.setOpaque(false);
      github.setToolTipText(aboutLink);
      github.setAlignmentX(Component.CENTER_ALIGNMENT);
      github.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent evt)
         {
            if (aboutURI != null)
            {
               if (Desktop.isDesktopSupported())
               {
                  try
                  {
                     Desktop.getDesktop().browse(aboutURI);
                  } catch (IOException e) {
                     System.err.println("Warning: could not open About URI.");
                  }
               }
               else
               {
                  System.err.println("Warning: could not open About URI; desktop not supported.");
               }
            }
         }
      });
      
      aboutDialog.getContentPane().add(github);

      JLabel yobotics = new JLabel("Originally developed at Yobotics, Inc. from 2000-2010.");
      yobotics.setFont(new Font("DTL Nobel", Font.PLAIN, 20));
      yobotics.setAlignmentX(Component.CENTER_ALIGNMENT);
      yobotics.setBorder(BorderFactory.createEmptyBorder(10, 0, 10, 0));
      aboutDialog.getContentPane().add(yobotics);

      JLabel ihmc = new JLabel("Now developed at IHMC from 2002 to the present.");
      ihmc.setFont(new Font("DTL Nobel", Font.PLAIN, 20));
      ihmc.setAlignmentX(Component.CENTER_ALIGNMENT);
      aboutDialog.getContentPane().add(ihmc);
      
      aboutDialog.setSize(new Dimension(600, 375));
      aboutDialog.validate();
      aboutDialog.setResizable(false);
      aboutDialog.setVisible(true);
   }

   public void closeAndDispose()
   {
      parentJFrame = null;
   }

}
