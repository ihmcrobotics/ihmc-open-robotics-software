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
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

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
   private Plotter rawPlotter = null;
   private Plotter filteredPlotter = null;
   private Container fftPlotContainer = null;
   private int dataSize = 0;

   private AlphaFilteredYoVariable filteredYoVariable = new AlphaFilteredYoVariable("FilteredMicrophoneData", null, 0.95);

   public DrillDetectorGui()
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

      PlotterPanel rawPlotterPanel = new PlotterPanel();
      GridBagConstraints rawLayout = new GridBagConstraints();
      rawLayout.gridx = 0;
      rawLayout.gridy = 1;
      rawLayout.weightx = 1;
      rawLayout.fill = GridBagConstraints.HORIZONTAL;
      soundDetectorGUI.add(rawPlotterPanel, rawLayout);

      rawPlotter = rawPlotterPanel.getPlotter();
      rawPlotter.setXoffset(320);
      rawPlotter.setYoffset(80);
      rawPlotter.setRange(200);

      PlotterPanel filteredPlotterPanel = new PlotterPanel();
      GridBagConstraints filteredLayout = new GridBagConstraints();
      filteredLayout.gridx = 0;
      filteredLayout.gridy = 2;
      filteredLayout.weightx = 1;
      filteredLayout.fill = GridBagConstraints.HORIZONTAL;
      soundDetectorGUI.add(filteredPlotterPanel, filteredLayout);

      filteredPlotter = filteredPlotterPanel.getPlotter();
      filteredPlotter.setXoffset(320);
      filteredPlotter.setYoffset(80);
      filteredPlotter.setRange(200);

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
      double rawValue = result.isOn ? 100.0 : 0.0;

      Point2d pRaw = new Point2d(x, rawValue);
      PointArtifact paRaw = new PointArtifact("drillOn_" + x, pRaw);
      rawPlotter.addArtifact(paRaw);

      filteredYoVariable.update(rawValue);
      double filteredValue = filteredYoVariable.getDoubleValue();

      Point2d pFiltered = new Point2d(x, filteredValue);
      PointArtifact paFiltered = new PointArtifact("filteredDrillOn_" + x, pFiltered);
      filteredPlotter.addArtifact(paFiltered);

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