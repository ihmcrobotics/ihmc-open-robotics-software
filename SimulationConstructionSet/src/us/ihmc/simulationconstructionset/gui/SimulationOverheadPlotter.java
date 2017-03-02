package us.ihmc.simulationconstructionset.gui;

import java.awt.BorderLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;

import us.ihmc.graphicsDescription.plotting.PlotterColors;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterLegendPanel;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.PlaybackListener;

public class SimulationOverheadPlotter implements PlaybackListener
{
   private final Plotter plotter;
   private final PlotterLegendPanel legendPanel;
   private final PlotterShowHideMenu plotterShowHideMenu;

   private JMenuBar menuBar;
   private JCheckBoxMenuItem trackBodyCB;

   private DoubleYoVariable xVariableToTrack, yVariableToTrack, yawVariableToTrack;

   public SimulationOverheadPlotter()
   {
      plotter = new Plotter(PlotterColors.javaFXStyle(), false);
      legendPanel = plotter.createPlotterLegendPanel();
      plotter.setViewRange(1.0);
      buildMenuBar();

      plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);
   }

   public JFrame createAndDisplayJFrame()
   {
      JFrame jFrame = new JFrame("Simulation Overhead Plotter");
      jFrame.addWindowListener(new WindowAdapter()
      {
         @Override
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
   
   public void setYawVariableToTrack(DoubleYoVariable yawVariableToTrack)
   {
      this.yawVariableToTrack = yawVariableToTrack;
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
      return plotter.getJPanel();
   }

   public JPanel getJPanelWithCheckBoxes()
   {
      JPanel jPanel = new JPanel(new BorderLayout());
      jPanel.add(plotter.getJPanel(), BorderLayout.CENTER);
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
      plotter.update();
   }

   private void recenterIfTrackingIsSelected()
   {
      if (trackBodyCB.isSelected())
      {
         if (xVariableToTrack != null)
            plotter.setFocusPointX(xVariableToTrack.getDoubleValue());
         if (yVariableToTrack != null)
            plotter.setFocusPointY(yVariableToTrack.getDoubleValue());
         if (yawVariableToTrack != null)
            plotter.setFocusOrientationYaw(yawVariableToTrack.getDoubleValue());
      }
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      update();
   }

   @Override
   public void play(double realTimeRate)
   {
   }

   @Override
   public void stop()
   {
   }

   public PlotterShowHideMenu getMenuPanel()
   {
      return plotterShowHideMenu;
   }
}
