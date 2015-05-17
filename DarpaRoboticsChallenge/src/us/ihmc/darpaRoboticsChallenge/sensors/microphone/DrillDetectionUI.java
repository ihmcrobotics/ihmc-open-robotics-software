package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.awt.Color;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.vecmath.Point2d;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.simulationconstructionset.gui.FFTPlotter;

public class DrillDetectionUI
{
   private final DrillDetectionThread detectorThread = new DrillDetectionThread(new DrillDetectionAlgorithmSimple())
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
   private Container fftPlotContainer = null;
   private Plotter boolPlotter = null;
   private Plotter[] doublePlotter = null;
   private int dataSize = 0;

   public DrillDetectionUI()
   {
      System.out.println("Creating the UI");

      JFrame frame = new JFrame("Drill Detection UI");
      frame.setContentPane(createContentPane());
      frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
      frame.setSize(1000, 900);
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
      soundDetectorGUI = new JPanel(new GridBagLayout());
      soundDetectorGUI.setOpaque(true);

      fftPlotContainer = new Container();
      fftPlotContainer.setBackground(Color.blue);
      GridBagConstraints fftLayout = new GridBagConstraints();
      fftLayout.gridx = 0;
      fftLayout.gridy = 0;
      fftLayout.weightx = 1;
      fftLayout.fill = GridBagConstraints.HORIZONTAL;
      soundDetectorGUI.add(fftPlotContainer, fftLayout);

      PlotterPanel boolPlotterPanel = new PlotterPanel();
      GridBagConstraints boolPlotterLayout = new GridBagConstraints();
      boolPlotterLayout.gridx = 0;
      boolPlotterLayout.gridy = 1;
      boolPlotterLayout.weightx = 1;
      boolPlotterLayout.fill = GridBagConstraints.HORIZONTAL;
      soundDetectorGUI.add(boolPlotterPanel, boolPlotterLayout);

      boolPlotter = boolPlotterPanel.getPlotter();
      boolPlotter.setXoffset(320);
      boolPlotter.setYoffset(80);
      boolPlotter.setRange(200);

      doublePlotter = new Plotter[1];
      for (int i = 0; i < doublePlotter.length; i++)
      {
         PlotterPanel doublePlotterPanel = new PlotterPanel();
         GridBagConstraints doublePlotterLayout = new GridBagConstraints();
         doublePlotterLayout.gridx = 0;
         doublePlotterLayout.gridy = 2 + i;
         doublePlotterLayout.weightx = 1;
         doublePlotterLayout.fill = GridBagConstraints.HORIZONTAL;
         soundDetectorGUI.add(doublePlotterPanel, doublePlotterLayout);

         doublePlotter[i] = doublePlotterPanel.getPlotter();
         doublePlotter[i].setXoffset(320);
         doublePlotter[i].setYoffset(80);
         doublePlotter[i].setRange(200);
      }

      return soundDetectorGUI;
   }

   private void processDrillDetectionResult(DrillDetectionResult result)
   {
      soundDetectorGUI.remove(fftPlotContainer);
      FFTPlotter plot = new FFTPlotter(result.bodeData, "", "(Hz)", "(dB)", "(deg)");
      fftPlotContainer = plot.getContentPane();
      GridBagConstraints fftLayout = new GridBagConstraints();
      fftLayout.gridx = 0;
      fftLayout.gridy = 0;
      fftLayout.weightx = 1;
      fftLayout.fill = GridBagConstraints.HORIZONTAL;
      soundDetectorGUI.add(fftPlotContainer, fftLayout);

      int x = dataSize++;
      double rawValue = result.isOn ? 150.0 : 0.0;

      Point2d pRaw = new Point2d(x, rawValue);
      PointArtifact paRaw = new PointArtifact("drillOn_" + x, pRaw);
      boolPlotter.addArtifact(paRaw);

      if (doublePlotter.length != result.averageValues.length)
      {
         System.err.println("Number of plotters != array size");
         return;
      }

      for (int i = 0; i < doublePlotter.length; i++)
      {
         Point2d pAverage = new Point2d(x, result.averageValues[i]);
         PointArtifact paAverage = new PointArtifact("average_" + i + "_" + x, pAverage);
         doublePlotter[i].addArtifact(paAverage);
      }

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
         public void run() { new DrillDetectionUI(); }
      });
   }
}