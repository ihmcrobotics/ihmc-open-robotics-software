package us.ihmc.simulationconstructionset.plotting;

import java.awt.BorderLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterLegendPanel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.PlaybackListener;

public class SimulationOverheadPlotter implements PlaybackListener
{
   private final Plotter plotter = new Plotter();
   private final PlotterLegendPanel legendPanel = plotter.createPlotterLegendPanel();

   private JMenuBar menuBar;
   private JCheckBoxMenuItem trackBodyCB;

   private DoubleYoVariable xVariableToTrack, yVariableToTrack;
   
   public SimulationOverheadPlotter()
   {
      plotter.setRange(1.0);
      buildMenuBar();
   }
   
   public JFrame createAndDisplayJFrame()
   {
      JFrame jFrame = new JFrame("Simulation Overhead Plotter");
      jFrame.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
//          System.exit(0);
         }
      });

      jFrame.setJMenuBar(menuBar);

      JPanel plotterAndLegendPanel = plotter.createAndAttachPlotterLegendPanel();

//    jFrame.getContentPane().add(plotter, BorderLayout.CENTER);
      jFrame.getContentPane().add(plotterAndLegendPanel, BorderLayout.CENTER);

      jFrame.pack();
      jFrame.setVisible(true);
      
      return jFrame;
   }
   
   public void setXVariableToTrack(DoubleYoVariable xVariableToTrack)
   {
      this.xVariableToTrack = xVariableToTrack;
   }
   
   public void setYVariableToTrack(DoubleYoVariable yVariableToTrack)
   {
      this.yVariableToTrack = yVariableToTrack;
   } 
   
   public void setDrawHistory(boolean drawHistory)
   {
      plotter.setDrawHistory(drawHistory);
   }
   
   private JMenuBar buildMenuBar()
   {
      // Create the menu bar.
      menuBar = new JMenuBar();

      // Build the first menu.
      JMenu menu = new JMenu("Options");
      menu.getAccessibleContext().setAccessibleDescription("Options");
      menuBar.add(menu);

      // a group of check box menu items
      trackBodyCB = new JCheckBoxMenuItem("track body");
      trackBodyCB.setSelected(true);
      menu.add(trackBodyCB);

      return menuBar;
   }
   
   public JMenuBar getJMenuBar()
   {
      return menuBar;
   }

   public Plotter getPlotter()
   {
      return plotter;
   }
   
   public JPanel getJPanel()
   {
      return plotter;
   }
   
   public JPanel getJPanelWithCheckBoxes()
   {
      JPanel jPanel = new JPanel(new BorderLayout());
      jPanel.add(plotter, BorderLayout.CENTER);
      jPanel.add(trackBodyCB, BorderLayout.NORTH);
      return jPanel;
   }

   public JPanel getJPanelKey()
   {
      return legendPanel;
   }

   public void update()
   {
      recenterIfTrackingIsSelected();
      plotter.repaint();
   }

   private void recenterIfTrackingIsSelected()
   {
      if (trackBodyCB.isSelected())
      {
         if (xVariableToTrack != null) plotter.setXoffset(xVariableToTrack.getDoubleValue());
         if (yVariableToTrack != null) plotter.setYoffset(yVariableToTrack.getDoubleValue());
      }
   }

   public void indexChanged(int newIndex, double newTime)
   {
      update();
   }

   public void play(double realTimeRate)
   {
   }

   public void stop()
   {
   }
}
