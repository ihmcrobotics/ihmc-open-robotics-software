package us.ihmc.plotting;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;

import java.io.File;
import java.io.IOException;

import java.text.DecimalFormat;

import java.util.Vector;

import javax.imageio.ImageIO;

import javax.swing.*;

import java.util.ArrayList;

public class PlotterPanel extends JPanel
{
   protected Plotter plotter;
   protected JTextArea eventTA;
   protected boolean plotMovement = false;

   protected JRadioButtonMenuItem gpsRB;
   protected JButton recenterBtn;
   public DecimalFormat twoPlaceDecimalFormatter = new DecimalFormat("00");

   public PlotterPanel()
   {
      plotter = new Plotter();

      GridBagConstraints gridBagConstraints = new GridBagConstraints();
      this.setLayout(new GridBagLayout());

      gridBagConstraints.gridx = 0;
      gridBagConstraints.gridy = 0;
      gridBagConstraints.fill = GridBagConstraints.BOTH;
      gridBagConstraints.gridwidth = 5;
      gridBagConstraints.gridheight = 5;
      gridBagConstraints.weightx = 1;
      gridBagConstraints.weighty = 1;
      this.add(plotter, gridBagConstraints);
   }

   public Plotter getPlotter()
   {
      return plotter;
   }

   public void setPlotMovement(boolean choice)
   {
      plotMovement = choice;
   }

   public boolean isPlottingMovement()
   {
      return plotMovement;
   }

// public void setdatum(GPSPosition datum)
// {
//         _datum = datum;
//         _gpsRB.setSelected(true);
// }

   public Vector getMenus()
   {
      JMenu menu;
      JMenuItem menuItem;
      JRadioButtonMenuItem rbMenuItem;
      JCheckBoxMenuItem cbMenuItem;

      Vector menus = new Vector();

      // Build the first menu.
      menu = new JMenu("OverHead View");
      menu.getAccessibleContext().setAccessibleDescription("Overhead 2D display options");

      // a group of JMenuItems
      menuItem = new JMenuItem("Re-center");
      menuItem.getAccessibleContext().setAccessibleDescription("Center 2D display on current selected position");
      menuItem.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent ae)
         {
            double newX = plotter.getSelectedX();
            double newY = plotter.getSelectedY();
            plotter.setXoffset(newX);
            plotter.setYoffset(newY);
         }
      });
      menu.add(menuItem);

      // save plotted movement
      menuItem = new JMenuItem("Save movement plot");
      menuItem.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent ae)
         {
            Artifact gps = plotter.getArtifact("gps");
            Artifact odom = plotter.getArtifact("odom");

            // saveMovementPlot((PointArtifact)gps, (PointArtifact)odom);
         }
      });
      menu.add(menuItem);


      // a group of check box menu items
      cbMenuItem = new JCheckBoxMenuItem("plot movement");
      cbMenuItem.addItemListener(new ItemListener()
      {
         public void itemStateChanged(ItemEvent ie)
         {
            if (ie.getStateChange() == ItemEvent.SELECTED)
            {
               plotMovement = true;
            }
            else
            {
               plotMovement = false;
               ArrayList<Artifact> artifacts = plotter.getArtifacts();
               for (int i = 0; i < artifacts.size(); i++)
               {
                  Artifact artifact = (Artifact) artifacts.get(i);
                  if (artifact.getID().endsWith("_gps") || artifact.getID().endsWith("_odom"))
                  {
                     plotter.removeArtifact(artifact.getID());
                  }
               }
            }
         }
      });
      menu.add(cbMenuItem);
      menu.addSeparator();
      menu.add(new JLabel("Reference"));

      ButtonGroup group = new ButtonGroup();

//    rbMenuItem = new JRadioButtonMenuItem("X-Y");
//    rbMenuItem.setSelected(true);
//    rbMenuItem.addActionListener(this);
//    group.add(rbMenuItem);
//    menu.add(rbMenuItem);
//    rbMenuItem = new JRadioButtonMenuItem("X-Z");
//    rbMenuItem.addActionListener(this);
//    group.add(rbMenuItem);
//    menu.add(rbMenuItem);
//    rbMenuItem = new JRadioButtonMenuItem("Y-Z");
//    rbMenuItem.addActionListener(this);
//    group.add(rbMenuItem);
//    menu.add(rbMenuItem);
      gpsRB = new JRadioButtonMenuItem("GPS");
      gpsRB.addItemListener(new ItemListener()
      {
         public void itemStateChanged(ItemEvent ie)
         {
            if (ie.getStateChange() == ItemEvent.SELECTED)
            {
//             //convert to gps
//             Coordinate coord = new Coordinate(((range / 2) * -1), ((range / 2) * -1), Coordinate.METER);
//             coord = new Coordinate((range / 2), (range / 2), Coordinate.METER);
            }

         }
      });
      group.add(gpsRB);
      menu.add(gpsRB);

      menus.add(menu);

      return menus;
   }

   public void actionPerformed(ActionEvent e)
   {
      if (e.getActionCommand().equals("X-Y"))
      {
         plotter.setOrientation(Plottable.X_Y);
      }
      else if (e.getActionCommand().equals("X-Z"))
      {
         plotter.setOrientation(Plottable.X_Z);
      }
      else if (e.getActionCommand().equals("Y-Z"))
      {
         plotter.setOrientation(Plottable.Y_Z);
      }
   }

   public void setUpdateDelayInMillis(long timeInMillis)
   {
      plotter.setUpdateDelayInMillis(timeInMillis);
   }

   public long getUpdateDelayInMillis()
   {
      return plotter.getUpdateDelayInMillis();
   }

   /**
    * Return the unique identifier of this observer.
    *
    * @return   String  containing the unique ID of this observer.
    */
   public String getID()
   {
      return ("Robot Client");
   }

// public void saveMovementPlot(PointArtifact gps, PointArtifact odom)
// {
//
//     String date = FileUtil.getDateString();
//     String time = FileUtil.getTimeString();
//     String filename = date + "_" + time + "_gps.plot";
//     try
//     {
//         PrintWriter pw = FileUtil.getFileWriter(filename);
//         gps.save(pw);
//         pw.flush();
//         pw.close();
//         System.out.println("Saved gps to " + filename);
//
//         filename = date + "_" + time + "_odom.plot";
//         pw = FileUtil.getFileWriter(filename);
//         odom.save(pw);
//         pw.flush();
//         pw.close();
//         System.out.println("Saved odom to " + filename);
//     }
//     catch(Exception e)
//     {
//         e.printStackTrace();
//     }
// }

   // test driver //////////////////////////////////////////////////
   public static void main(String[] args)
   {
      PlotterPanel p = new PlotterPanel();
      if (args.length >= 1)
      {
         BufferedImage image = null;
         try
         {
            image = ImageIO.read(new File(args[0]));
         }
         catch (IOException e)
         {
            JOptionPane.showMessageDialog(null, e.getMessage(), "", JOptionPane.ERROR_MESSAGE);
            System.exit(1);
         }

         p.getPlotter().setBackgroundImage(image);
         p.getPlotter().setRangeLimit(20, (double) (image.getWidth() / 20.0), -10.0, 10.0, 10.0, -10.0);
      }

      JFrame f = new JFrame("Plotter Panel");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });

//    f.setJMenuBar((JMenuBar)p.getMenus().get(0));
      f.getContentPane().add(new JScrollPane(p), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);

   }
}
