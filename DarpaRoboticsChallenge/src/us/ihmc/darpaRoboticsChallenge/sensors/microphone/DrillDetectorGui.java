package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.vecmath.Point2d;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.simulationconstructionset.gui.FFTPlotter;

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
      public void onDrillDetectionResult(final DrillDetectionResult result)
      {
         Runnable processResult = new Runnable()
         {
            @Override
            public void run() { processDrillDetectionResult(result); }
         };

         SwingUtilities.invokeLater(processResult);
      }
   };

   private JPanel soundDetectorGUI = null;
   private Plotter plotter = null;
   private int plotterDataSize = 0;
   private Container fftPlotContainer = null;

   public DrillDetectorGui()
   {
      System.out.println("Creating the UI");

      JFrame frame = new JFrame("Drill Detection UI");
      frame.setContentPane(createContentPane());
      frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
      frame.setSize(800, 800);
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
      soundDetectorGUI = new JPanel(new BorderLayout());
      soundDetectorGUI.setOpaque(true);

      PlotterPanel plotterPanel = new PlotterPanel();
      plotter = plotterPanel.getPlotter();
      plotter.setXoffset(220);
      plotter.setYoffset(80);
      plotter.setRange(200);
      soundDetectorGUI.add(plotterPanel, BorderLayout.PAGE_START);

      fftPlotContainer = new Container();
      fftPlotContainer.setBackground(Color.blue);
      soundDetectorGUI.add(fftPlotContainer, BorderLayout.CENTER);

      return soundDetectorGUI;
   }

   private void processDrillDetectionResult(DrillDetectionResult result)
   {
      int data = result.isOn ? 100 : 0;

      int x = plotterDataSize++;
      Point2d p = new Point2d(x, data);
      PointArtifact pa = new PointArtifact("drillOn_" + x, p);
      plotter.addArtifact(pa);

      soundDetectorGUI.remove(fftPlotContainer);
      FFTPlotter plot = new FFTPlotter(result.bodeData, "FFT Plot", "(Hz)", "(dB)", "(deg)");
      fftPlotContainer = plot.getContentPane();
      soundDetectorGUI.add(fftPlotContainer, BorderLayout.CENTER);

      soundDetectorGUI.revalidate();
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