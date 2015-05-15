package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.vecmath.Point2d;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.plotting.shapes.PointArtifact;

/**
 * Sound Detector Gui
 * 
 * Press start to listen to microphone input and run the rotozip sound detector. "Rotozip ON!!!!" should show up on the gui when the sound detector notices it.
 * 
 * @author Will
 *
 */

public class DrillDetectorGui
{
   private final DrillDetectorThread detectorThread = new DrillDetectorThread()
   {
      @Override
      public void onDrillDetectionResult(final boolean isDrillOn)
      {
         Runnable processResult = new Runnable()
         {
            @Override
            public void run() { processDrillDetectionResult(isDrillOn); }
         };

         SwingUtilities.invokeLater(processResult);
      }
   };

   private Plotter plotter = null;
   private int plotterDataSize = 0;

   public DrillDetectorGui()
   {
      System.out.println("Creating the UI");

      JFrame frame = new JFrame("Drill Detection UI");
      frame.setContentPane(createContentPane());
      frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
      frame.setSize(700, 400);
      frame.setVisible(true);

      Runtime.getRuntime().addShutdownHook(new Thread()
      {
         @Override
         public void run() { onShutdown(); }
      });

      detectorThread.start();
   }

   private JPanel createContentPane()
   {
      JPanel soundDetectorGUI = new JPanel(new BorderLayout());
      soundDetectorGUI.setOpaque(true);

      PlotterPanel plotterPanel = new PlotterPanel();
      plotter = plotterPanel.getPlotter();
      plotter.setXoffset(140);
      plotter.setYoffset(60);
      plotter.setRange(200);
      soundDetectorGUI.add(plotterPanel, BorderLayout.CENTER);

      return soundDetectorGUI;
   }
   
   private void processDrillDetectionResult(boolean isDrillOn)
   {
      int data = isDrillOn ? 70 : 0;

      int x = plotterDataSize++;
      Point2d p = new Point2d(x, data);
      PointArtifact pa = new PointArtifact("drillOn_" + x, p);
      plotter.addArtifact(pa);
   }

   private void onShutdown()
   {
      System.out.println("Killing drill detection thread...");
      detectorThread.shutdown();

      try { detectorThread.join(); }
      catch (Exception ignored) { }

      System.out.println("Terminating the UI");
   }

   public static void main(String[] args)
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run() { new DrillDetectorGui(); }
      });
   }
}