package us.ihmc.simulationconstructionset.util.graphs;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.apache.batik.dom.GenericDOMImplementation;
import org.apache.batik.svggen.SVGGraphics2D;
import org.apache.pdfbox.exceptions.COSVisitorException;
import org.apache.pdfbox.pdmodel.PDDocument;
import org.apache.pdfbox.pdmodel.PDPage;
import org.apache.pdfbox.pdmodel.edit.PDPageContentStream;
import org.apache.pdfbox.pdmodel.graphics.xobject.PDPixelMap;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.LogarithmicAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.chart.renderer.xy.XYShapeRenderer;
import org.jfree.chart.title.LegendTitle;
import org.jfree.data.Range;
import org.jfree.data.xy.XYSeriesCollection;
import org.w3c.dom.DOMImplementation;
import org.w3c.dom.Document;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.DataBufferEntry;

public class JFreeGraph extends JPanel
{
   private static final long serialVersionUID = -2146727132718915859L;
   private static final Stroke DEFAULT_GRIDLINE_STROKE = new BasicStroke(6f);
   private JFreeChart graph;
   private final XYSeriesCollection xySeriesCollection = new XYSeriesCollection();
   private String xLabel;
   private String yLabel;
   ArrayList<JFreePlot> plots = new ArrayList<JFreePlot>();
   private String title;
   BufferedImage image;

   private AxisScaling xAxisScaling;
   private AxisScaling yAxisScaling;

   public enum AxisScaling
   {
      CONSTANT, LOGARITHMIC
   }

   public JFreeChart getJFreeChart()
   {
      return graph;
   }

   public void setJFreeChart(JFreeChart graph)
   {
      this.graph = graph;
   }

   @Override
   public JFreeGraph clone()
   {
      JFreeGraph returnGraph = new JFreeGraph(title, xLabel, yLabel, plots);
      returnGraph.setJFreeChart(this.getJFreeChart());

      return returnGraph;
   }

   public JFreeGraph(String title)

   {
      this(title, "x", "y");
   }

   public JFreeGraph(String title, String xLabel, String yLabel)
   {
      this.title = title;
      this.xLabel = xLabel;
      this.yLabel = yLabel;
      setUpGraph();
   }

   public JFreeGraph(String title, String xLabel, String yLabel, JFreePlot plot)
   {
      this(title, xLabel, yLabel);
      plots.add(plot);
      setUpGraph();
   }

   public JFreeGraph(String title, String xLabel, String yLabel, ArrayList<JFreePlot> plots)
   {
      this.title = title;
      this.xLabel = xLabel;
      this.yLabel = yLabel;
      this.plots = plots;
      setUpGraph();
   }

   private void setUpGraph()
   {
      xySeriesCollection.removeAllSeries();

      for (JFreePlot plot : plots)
      {
         xySeriesCollection.addSeries(plot);
      }

      graph = ChartFactory.createXYLineChart(title, // Title
            xLabel, // x-axis Label
            yLabel, // y-axis Label
            xySeriesCollection, // Dataset
            PlotOrientation.VERTICAL, // Plot Orientation
            false, // Show Legend
            true, // Use tooltips
            false // Configure chart to generate URLs?
      );

      // set style for graph
      setUpVisuals();

      // ChartPanel chartpanel = new ChartPanel(graph, true, true, true, true, true);

      // ChartPanel chartpanel = new ChartPanel(graph, 320, 240, 320, 240, 320, 240, true, true, true, true, true, true, true);
   }

   @Override
   public void paintComponent(Graphics graphics)
   {
      image = graph.createBufferedImage(this.getWidth(), this.getHeight(), this.getWidth(), this.getHeight(), null);
      graphics.drawImage(image, 0, 0, this);
   }

   public void addPlot(JFreePlot plot)
   {
      plots.add(plot);
      setUpGraph();
   }

   public void setTitle(String title)
   {
      this.title = title;
      setUpGraph();
   }

   private void setXAxis()
   {
      ValueAxis axis;
      switch (xAxisScaling)
      {
      case LOGARITHMIC:
         axis = new LogarithmicAxis(xLabel);
         break;
      case CONSTANT:
      default:
         axis = new NumberAxis(xLabel);
         break;
      }
      graph.getXYPlot().setDomainAxis(0, axis);
   }

   private void setYAxis()
   {
      ValueAxis axis;
      switch (yAxisScaling)
      {
      case LOGARITHMIC:
         axis = new LogarithmicAxis(xLabel);
         break;
      case CONSTANT:
      default:
         axis = new NumberAxis(xLabel);
         break;
      }

      graph.getXYPlot().setRangeAxis(0, axis);
   }

   public void setXAxisLabel(String label)
   {
      this.xLabel = label;
      setXAxis();

   }

   public void setYAxisLabel(String label)
   {
      this.yLabel = label;
      setYAxis();

   }

   public void setXaxisScaling(AxisScaling scaling)
   {
      this.xAxisScaling = scaling;
      setXAxis();
   }

   public void setYaxisScaling(AxisScaling scaling)
   {
      this.yAxisScaling = scaling;
      setYAxis();
   }

   public void setYAxisTickUnit(double tickUnit)
   {
      // this needs to be fixed, if setUpGraph is called after this then it overwrites it.
      XYPlot xyplot = graph.getXYPlot();
      NumberAxis numberAxis = (NumberAxis) xyplot.getRangeAxis();
      numberAxis.setTickUnit(new NumberTickUnit(tickUnit));
   }

   public void setXAxisTickUnit(double tickUnit)
   {
      // this needs to be fixed, if setUpGraph is called after this then it overwrites it.
      XYPlot xyplot = graph.getXYPlot();
      NumberAxis numberAxis = (NumberAxis) xyplot.getDomainAxis();
      numberAxis.setTickUnit(new NumberTickUnit(tickUnit));

   }

   public void setXAxisRange(double start, double end)
   {
      // this needs to be fixed, if setUpGraph is called after this then it overwrites it.
      XYPlot xyplot = graph.getXYPlot();
      NumberAxis numberAxis = (NumberAxis) xyplot.getDomainAxis();
      numberAxis.setRange(new Range(start, end));

   }

   public void setYAxisRange(double start, double end)
   {
      // this needs to be fixed, if setUpGraph is called after this then it overwrites it.
      XYPlot xyplot = graph.getXYPlot();
      NumberAxis numberAxis = (NumberAxis) xyplot.getRangeAxis();
      numberAxis.setRange(new Range(start, end));

   }

   private void setPlotColor(int seriesNumber, Color color)
   {
      XYItemRenderer renderer = graph.getXYPlot().getRenderer();
      renderer.setSeriesPaint(seriesNumber, color);
   }

   private void setPlotStroke(int seriesNumber, BasicStroke stroke)
   {
      XYItemRenderer renderer = graph.getXYPlot().getRenderer();
      renderer.setSeriesStroke(seriesNumber, stroke);
   }

   public JFreePlot getPlot(String name)
   {
      for (JFreePlot plot : plots)
      {
         if (plot.getName().equals(name))
            return plot;
      }

      return null;
   }

   public void enableGrid(boolean enable)
   {
      if (enable)
      {
         BasicStroke dotted = new BasicStroke(1.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1.0f, new float[] {1.0f, 6.0f}, 0.0f);
         graph.getXYPlot().setDomainGridlinesVisible(true);
         graph.getXYPlot().setRangeGridlinesVisible(true);
         graph.getXYPlot().setRangeGridlinePaint(Color.black);
         graph.getXYPlot().setDomainGridlinePaint(Color.black);
         graph.getXYPlot().setRangeGridlineStroke(dotted);
         graph.getXYPlot().setDomainGridlineStroke(dotted);

      }
      else
      {
         graph.getXYPlot().setDomainGridlinesVisible(false);
         graph.getXYPlot().setRangeGridlinesVisible(false);
      }
   }

   public static JFreeGraph createDataVsTimeGraph(DataBufferEntry timeEntry, DataBufferEntry dataEntry)
   {
      return createDataVsTimeGraph(timeEntry, dataEntry, Color.BLACK);
   }

   public static JFreeGraph createDataVsTimeGraph(DataBufferEntry timeEntry, DataBufferEntry dataEntry, Color plotColor)
   {
      String variableName = dataEntry.getVariable().getName();
      JFreePlot plot = new JFreePlot("time vs " + variableName, timeEntry, dataEntry);
      plot.setIsScatterPlot(false);
      plot.setColor(plotColor);
      
      JFreeGraph graph = new JFreeGraph(variableName, "time", variableName);
      graph.addPlot(plot);

      return graph;
   }

   public static JFreeGraph createDataOneVsDataTwoGraph(DataBufferEntry dataOneEntry, DataBufferEntry dataTwoEntry)
   {
      return createDataOneVsDataTwoGraph(dataOneEntry, dataTwoEntry, Color.BLACK);
   }

   public static JFreeGraph createDataOneVsDataTwoGraph(DataBufferEntry dataOneEntry, DataBufferEntry dataTwoEntry, Color plotColor)
   {
      String dataOneVariableName = dataOneEntry.getVariable().getName();
      String dataTwoVariableName = dataTwoEntry.getVariable().getName();
      JFreePlot plot = new JFreePlot((dataOneVariableName + "_Vs_" + dataTwoVariableName), dataOneEntry, dataTwoEntry, false, true);
      plot.setIsScatterPlot(true);
      plot.setColor(plotColor);

      JFreeGraph graph = new JFreeGraph((dataOneVariableName + "_Vs_" + dataTwoVariableName), dataOneVariableName, dataTwoVariableName);
      graph.addPlot(plot);

      return graph;
   }

   void saveToSVG(File svgFileName) throws IOException
   {
      DOMImplementation domImpl = GenericDOMImplementation.getDOMImplementation();
      Document document = domImpl.createDocument(null, "svg", null);

      // Create an instance of the SVG Generator
      SVGGraphics2D svgGenerator = new SVGGraphics2D(document);
      ChartPanel chartpanel = new ChartPanel(graph, true, true, true, false, true);

      JFrame tmpFrame = new JFrame();
      tmpFrame.getContentPane().add(chartpanel);
      tmpFrame.setSize(1024, 768);
      tmpFrame.setVisible(true);

      // draw the chart in the SVG generator
      graph.draw(svgGenerator, tmpFrame.getContentPane().getBounds());

      // Write svg file
      OutputStream outputStream = new FileOutputStream(svgFileName);
      Writer out = new OutputStreamWriter(outputStream, "UTF-8");
      svgGenerator.stream(out, true);
      outputStream.flush();
      outputStream.close();

      try
      {
         Thread.sleep(3000);
      }
      catch (InterruptedException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      tmpFrame.dispose();
   }

   public void saveToPDF(File pdfFileName)
   {
      int x = 1024, y = 768;
      saveToPDF(pdfFileName, x, y);
   }

   public void saveToPDF(File pdfFileName, int x, int y)
   {
      try
      {
         PDDocument document = new PDDocument();

         PDPage page = new PDPage();
         document.addPage(page);

         PDPageContentStream contentStream = new PDPageContentStream(document, page);

         BufferedImage image = new BufferedImage(x, y, BufferedImage.TYPE_INT_RGB);
         Graphics2D graphics2D = image.createGraphics();
         graph.draw(graphics2D, new Rectangle(x, y));
         graphics2D.dispose();

         PDPixelMap pixelMap = new PDPixelMap(document, image);

         contentStream.drawImage(pixelMap, x, y);
         contentStream.close();

         document.save(pdfFileName);

         document.close();
      }
      catch (IOException | COSVisitorException e)
      {
         PrintTools.error(this, "Could not save pdf to file " + pdfFileName);
      }
   }

   public void saveToJPG(File jpgFileName, int width, int height)
   {
      try
      {
         ChartUtilities.saveChartAsJPEG(jpgFileName, graph, width, height);
      }
      catch (IOException e)
      {
         System.err.println("Problem occurred creating chart.");
      }
   }

   private void setUpVisuals()
   {
      graph.getXYPlot().setDomainGridlinePaint(Color.BLACK);
      graph.getXYPlot().setDomainGridlinesVisible(false);
      graph.getXYPlot().setDomainGridlineStroke(new BasicStroke(3f));

      graph.getXYPlot().setRangeGridlinePaint(Color.BLACK);
      graph.getXYPlot().setRangeGridlinesVisible(false);
      graph.getXYPlot().setRangeGridlineStroke(new BasicStroke(3f));

      // set style for boarder line
      graph.getXYPlot().setOutlineVisible(false);
      graph.getXYPlot().setOutlinePaint(Color.BLACK);
      graph.getXYPlot().setOutlineStroke(DEFAULT_GRIDLINE_STROKE);

      // set color for inside of plot
      graph.getXYPlot().setBackgroundPaint(Color.WHITE);

      // set Color For outside of plot
      graph.setBackgroundPaint(Color.white);

      if ((!plots.isEmpty()) && (plots.get(0).isScatterPlot()))
      {

         XYShapeRenderer renderer = new XYShapeRenderer();
         Shape shape  = new Ellipse2D.Double(-4, -4, 8, 8);
         renderer.setBaseShape(shape);
         graph.getXYPlot().setRenderer(renderer);
      }

      for (int i = 0; i < plots.size(); i++)
      {
         JFreePlot plot = plots.get(i);
         setPlotColor(i, plot.getColor());
         setPlotStroke(i, plot.getBasicStroke());
      }

      //      graph.getLegend().setItemFont(new Font("SansSerif", Font.PLAIN, 16));
      graph.getTitle().setFont(new Font("SansSerif", Font.BOLD, 26));
      graph.getXYPlot().getDomainAxis().setTickLabelFont(new Font("SansSerif", Font.PLAIN, 20));
      graph.getXYPlot().getRangeAxis().setTickLabelFont(new Font("SansSerif", Font.PLAIN, 20));
      graph.getXYPlot().getDomainAxis().setLabelFont(new Font("SansSerif", Font.BOLD, 22));
      graph.getXYPlot().getRangeAxis().setLabelFont(new Font("SansSerif", Font.BOLD, 22));
   }

   public void addLegend(LegendTitle legend)
   {
      legend.setItemFont(new Font("SansSerif", Font.PLAIN, 22));
      graph.addLegend(legend);
   }
   
   public static void main(String[] args)
   {      
      int numberOfPoints = 1000;

      YoVariableRegistry registry = new YoVariableRegistry("Registry");
      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", registry);
      DoubleYoVariable variableTwo = new DoubleYoVariable("variableTwo", registry);
      
      DataBufferEntry dataOneEntry = new DataBufferEntry(variableOne, numberOfPoints);
      DataBufferEntry dataTwoEntry = new DataBufferEntry(variableTwo, numberOfPoints);
      
      for (int i=0; i<numberOfPoints; i++)
      {
         variableOne.set(Math.random());
         variableTwo.set(Math.random());
         dataOneEntry.setDataAtIndexToYoVariableValue(i);
         dataTwoEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      JFreeGraph jFreeGraphOne = JFreeGraph.createDataVsTimeGraph(dataOneEntry, dataTwoEntry, Color.orange);
      JFreeGraph jFreeGraphTwo = JFreeGraph.createDataOneVsDataTwoGraph(dataOneEntry, dataTwoEntry, Color.red);
      
      JFrame window = new JFrame("TimePlot");
      window.getContentPane().add(jFreeGraphOne);
      window.pack();
      window.setSize(600, 400);
      window.setVisible(true);
      
      window = new JFrame("DataPlot");
      window.getContentPane().add(jFreeGraphTwo);
      window.pack();
      window.setSize(600, 400);
      window.setVisible(true);    
   }

}
