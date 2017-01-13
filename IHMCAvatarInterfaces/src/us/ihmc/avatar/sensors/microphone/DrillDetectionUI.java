package us.ihmc.avatar.sensors.microphone;

import java.awt.Color;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.vecmath.Point2d;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.time.Millisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;

import us.ihmc.graphics3DDescription.plotting.artifact.PointListArtifact;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.gui.FFTPlotter;

public class DrillDetectionUI
{
   DrillDetectionAlgorithm detectionAlgorithm = new DrillDetectionCalibrationHelper();
   private final DrillDetectionThread detectorThread = new DrillDetectionThread(detectionAlgorithm)
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
   private int dataSize = 0;
   private int numBands;
   YoVariableRegistry registry = new YoVariableRegistry("registry");
   ArrayList<DoubleYoVariable> bandValues = new ArrayList<>();
   boolean shouldZero = true;
   double[] zeroValues;
   private final TimeSeriesCollection dataset;
   private int time = 0;

   public DrillDetectionUI()
   {
      System.out.println("Creating the UI");

      JFrame frame = new JFrame("Drill Detection UI");

      numBands = detectionAlgorithm.getNumReturnedBands();
      dataset = new TimeSeriesCollection();
      for (int i = 0; i < numBands; i++){
         DoubleYoVariable iVariable = new DoubleYoVariable(i+"AverageBandMagnitude", registry);
         bandValues.add(iVariable);
         dataset.addSeries(new TimeSeries(i+"th Band"));
      }
      zeroValues = new double[numBands];

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
      boolPlotter.setFocusPointX((double) 320);
      boolPlotter.setFocusPointY((double) 80);
      boolPlotter.setViewRange(200);


      JFreeChart bandGraph = ChartFactory.createTimeSeriesChart("Band Magnitudes", "Time", "Magnitude(dB)", dataset, true, false, false);
      final XYPlot plot = bandGraph.getXYPlot();
      ValueAxis axis = plot.getDomainAxis();
      axis.setAutoRange(true);
      axis.setFixedAutoRange(60000.0);  // 60 seconds
      axis = plot.getRangeAxis();
      axis.setRange(-25.0, 25.0);
      ChartPanel chartPanel = new ChartPanel(bandGraph);
      boolPlotterLayout.gridy++;
      soundDetectorGUI.add(chartPanel, boolPlotterLayout);

      JButton zero = new JButton("Zero Sound Differences");
      zero.addActionListener(new ActionListener() {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            shouldZero = true;
         }
      });
      boolPlotterLayout.gridy++;
      soundDetectorGUI.add(zero, boolPlotterLayout);

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
      PointListArtifact paRaw = new PointListArtifact("drillOn_" + x, pRaw);
      boolPlotter.addArtifact(paRaw);

      if (numBands != result.averageValues.length)
      {
         System.err.println("Number of plotters != array size");
         return;
      }

      Millisecond now = new Millisecond();
      double average = 0;
      for (int i = 0; i < numBands; i++)
      {
         bandValues.get(i).set(result.averageValues[i]);
         average+= result.averageValues[i];
      }
      if (numBands != 0) average /= numBands;

      for (int i = 0; i < numBands; i++)
      {
         if (shouldZero){
            zeroValues[i] = bandValues.get(i).getDoubleValue() - average;
         }
         dataset.getSeries(i).add(now, bandValues.get(i).getDoubleValue() - average - zeroValues[i]);
      }
      shouldZero = false;

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