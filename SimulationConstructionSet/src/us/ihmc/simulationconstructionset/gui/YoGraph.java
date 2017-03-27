package us.ihmc.simulationconstructionset.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.dnd.DropTarget;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;

import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.TransferHandler;
import javax.swing.border.BevelBorder;
import javax.swing.border.SoftBevelBorder;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.gui.dialogs.GraphPropertiesDialog;
import us.ihmc.simulationconstructionset.robotcommprotocol.GUISideCommandListener;

public class YoGraph extends JPanel implements MouseListener, MouseMotionListener, KeyListener, FocusListener
{
   private static final long serialVersionUID = -2801526140874071236L;

   private static final int DONT_PLOT_BOTTOM_PIXELS = 25;
   private static final int PIXELS_PER_BOTTOM_ROW = 14;    // 16;
   private static final int DONT_PLOT_TIMELINE_BOTTOM_PIXELS = 16;

   private GraphConfiguration graphConfiguration = new GraphConfiguration("default");

   protected static final int
      INDIVIDUAL_SCALING = GraphConfiguration.INDIVIDUAL_SCALING, AUTO_SCALING = GraphConfiguration.AUTO_SCALING,
      MANUAL_SCALING = GraphConfiguration.MANUAL_SCALING;
   protected static final int
      TIME_PLOT = GraphConfiguration.TIME_PLOT, PHASE_PLOT = GraphConfiguration.PHASE_PLOT;

   private final JFrame parentFrame;

   private final TimeDataHolder timeDataHolder;
   private final DataEntryHolder dataEntryHolder;
   private final GraphIndicesHolder graphIndicesHolder;
   private final YoGraphRemover yoGraphRemover;
   private final static int MAX_NUM_GRAPHS = 10;
   private final static int MAX_NUM_BASELINES = 6;
   private final static int VAR_NAME_SPACING_FOR_PRINT = 160;

   // public final static int VAR_SPACE = 110;
   // private final static int FONT_WIDTH = 5;

   // private final static int EMPTY_HEIGHT = 40;
   private final Color colors[] = new Color[YoGraph.MAX_NUM_GRAPHS];
   private final Color baseLineColors[] = new Color[YoGraph.MAX_NUM_BASELINES];
   
   private final ArrayList<DataEntry> entriesOnThisGraph;
   private final SelectedVariableHolder selectedVariableHolder;

   private double min = 0.0, max = 1.1;

   private int[] xData, yData;

   private final ArrayList<Integer> entryNamePaintWidths = new ArrayList<Integer>();
   private final ArrayList<Integer> entryNamePaintRows = new ArrayList<Integer>();
   private int totalEntryNamePaintRows = 1;
   private JPopupMenu popupMenu;
   private JMenuItem delete;
   private static int actionPerformedByDragAndDrop = -1;
   private static Object sourceOfDrag = null;
   private static Object recipientOfDrag = null;
   boolean hadFocus = false;
   private boolean showNameSpace = false, showBaseLines = false;
   private int focussedBaseLine = 0;

   public YoGraph(GraphIndicesHolder graphIndicesHolder, YoGraphRemover yoGraphRemover, SelectedVariableHolder holder, DataEntryHolder dataEntryHolder,
                  TimeDataHolder timeDataHolder, JFrame jFrame)
   {
      this.setName("YoGraph");
      
      this.selectedVariableHolder = holder;
      this.dataEntryHolder = dataEntryHolder;
      this.timeDataHolder = timeDataHolder;

      xData = new int[0];
      yData = new int[0];

      this.setBorder(new SoftBevelBorder(BevelBorder.LOWERED));

      this.graphIndicesHolder = graphIndicesHolder;
      this.yoGraphRemover = yoGraphRemover;
      this.parentFrame = jFrame;

      this.setOpaque(true);
      entriesOnThisGraph = new ArrayList<DataEntry>();

      colors[0] = new Color(0xa0, 0, 0);
      colors[1] = new Color(0, 0, 0xff);
      colors[2] = new Color(0, 0x80, 0);
      colors[3] = new Color(0, 0, 0);

      colors[4] = new Color(0x80, 0x80, 0x80);
      colors[5] = new Color(0x80, 0, 0x80);
      colors[6] = new Color(0, 0x80, 0x80);
      colors[7] = new Color(0x60, 0x60, 0);
      colors[8] = new Color(0xff, 0x50, 0x50);
      colors[9] = new Color(0x50, 0xff, 0xff);

      baseLineColors[0] = new Color(0x9370DB); // Purple
      baseLineColors[1] = new Color(0x3CB371); // Medium sea green
      baseLineColors[2] = Color.ORANGE;
      baseLineColors[3] = Color.ORANGE;
      baseLineColors[4] = Color.ORANGE;
      baseLineColors[5] = Color.ORANGE;

      this.addMouseListener(this);
      this.addMouseMotionListener(this);
      this.addKeyListener(this);
      this.setDropTarget(new DropTarget(this, new YoGraphTargetListener(this)));

      popupMenu = new ForcedRepaintPopupMenu();
      delete = new JMenuItem("Delete Graph");
      final YoGraph thisYoGraph = this;
      delete.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            thisYoGraph.yoGraphRemover.removeGraph(thisYoGraph);
         }
      });
      popupMenu.addFocusListener(this);
      this.addFocusListener(this);
      this.setTransferHandler(new YoGraphTransferHandler());
      this.showNameSpace = false;
   }

  
   public GraphConfiguration getGraphConfiguration()
   {
      return graphConfiguration;
   }

   protected int getScalingMethod()
   {
      return graphConfiguration.getScalingMethod();
   }

   protected void setScalingMethod(int method)
   {
      graphConfiguration.setScalingMethod(method);
   }

   protected int getPlotType()
   {
      return graphConfiguration.getPlotType();
   }

   protected void setPlotType(int type)
   {
      graphConfiguration.setPlotType(type);
   }

   protected double getManualMinScaling()
   {
      return graphConfiguration.getManualScalingMin();
   }

   protected double getManualMaxScaling()
   {
      return graphConfiguration.getManualScalingMax();
   }

   public void setShowBaseLines(boolean useBaseLine)
   {
      graphConfiguration.setShowBaseLines(useBaseLine);
   }

   public boolean getShowBaseLines()
   {
      return graphConfiguration.getShowBaseLines();
   }

   public void setBaseLine(double baseLine)
   {
      graphConfiguration.setBaseLine(baseLine);
   }

   public void setBaseLines(double baseLine1, double baseLine2)
   {
      graphConfiguration.setBaseLines(baseLine1, baseLine2);
   }

   public void setBaseLines(double[] baseLines)
   {
      graphConfiguration.setBaseLines(baseLines);
   }
   
   public void incrementBaseLine(int baseLineIndex, double scale)
   {
      if (baseLineIndex >= graphConfiguration.getBaseLines().length)
         baseLineIndex = 0;
      
      double min = this.getMin();
      double max = this.getMax();
      
      double range = max - min;
      double amountToIncrement = 0.01 * range * scale;

      graphConfiguration.incrementBaseLine(baseLineIndex, amountToIncrement);
   }
   
   public void zeroBaseLine(int baseLineIndex)
   {
      if (baseLineIndex >= graphConfiguration.getBaseLines().length)
         baseLineIndex = 0;

      graphConfiguration.setBaseLine(baseLineIndex, 0.0);
   }
   
   public void centerBaseLine(int baseLineIndex)
   {
      if (baseLineIndex >= graphConfiguration.getBaseLines().length)
         baseLineIndex = 0;
      
      double min = this.getMin();
      double max = this.getMax();
      
      double center = (max + min) / 2.0;

      graphConfiguration.setBaseLine(baseLineIndex, center);
   }

   public double[] getBaseLines()
   {
      return graphConfiguration.getBaseLines();
   }

   protected double getMax()
   {
      reCalcMinMax();

      return this.max;
   }

   protected double getMin()
   {
      reCalcMinMax();

      return this.min;
   }

   protected void setGraphConfiguration(GraphConfiguration graphConfiguration)
   {
      if (graphConfiguration == null)
         return;

      setManualScaling(graphConfiguration.getManualScalingMin(), graphConfiguration.getManualScalingMax());
      setScalingMethod(graphConfiguration.getScalingMethod());
      setPlotType(graphConfiguration.getPlotType());

      setShowBaseLines(graphConfiguration.getShowBaseLines());
      setBaseLines(graphConfiguration.getBaseLines());
   }

   protected void setManualScaling(double minScaling, double maxScaling)
   {
      graphConfiguration.setManualScalingMinMax(minScaling, maxScaling);
   }

   public ArrayList<DataEntry> getEntriesOnThisGraph()
   {
      return entriesOnThisGraph;
   }
   
   public boolean isEmpty()
   {
      return entriesOnThisGraph.isEmpty();
   }

   public void setInteractionEnable(boolean enable)
   {
      if (enable)
      {
         // First remove all listeners in case they're already there:
         // Could also use this.getListeners(MouseListener); and remove those attached...

         this.removeMouseListener(this);
         this.removeMouseMotionListener(this);
         this.removeKeyListener(this);

         // Then add them back.
         this.addMouseListener(this);
         this.addMouseMotionListener(this);
         this.addKeyListener(this);
      }

      else
      {    // Just disable them and assume no repeats...
         this.removeMouseListener(this);
         this.removeMouseMotionListener(this);
         this.removeKeyListener(this);
      }
   }

   public int getNumVars()
   {
      return this.entriesOnThisGraph.size();
   }

   private int previousGraphWidth;

   private void calculateRequiredEntryPaintWidthsAndRows()
   {
      FontMetrics fontMetrics = this.getFontMetrics(getFont());

      entryNamePaintWidths.clear();
      entryNamePaintRows.clear();

      int graphWidth = this.getWidth();
      previousGraphWidth = graphWidth;

      int cumulatedWidth = 0;
      int row = 0;

      for (DataEntry entry : entriesOnThisGraph)
      {
         int variableWidth = fontMetrics.stringWidth(entry.getVariableName());
         int variablePlusValueWidth = variableWidth + 120;

         if ((cumulatedWidth != 0) && (cumulatedWidth + variablePlusValueWidth > graphWidth))
         {
            row++;
            cumulatedWidth = 0;
         }

         cumulatedWidth += variablePlusValueWidth;

         entryNamePaintWidths.add(variablePlusValueWidth);
         entryNamePaintRows.add(row);
      }

      this.totalEntryNamePaintRows = row + 1;
   }

   public void addVariable(DataEntry entry)
   {
      if (entry == null)
      {
         return;
      }

      if (entriesOnThisGraph.size() >= YoGraph.MAX_NUM_GRAPHS)
         return;

      if (!entriesOnThisGraph.contains(entry))
         entriesOnThisGraph.add(entry);

      // this.repaint();
      // Do updateUI instead of just repaint whenever adding or deleting stuff.
      this.reCalcMinMax();
      calculateRequiredEntryPaintWidthsAndRows();

      this.updateUI();
   }

   public void addVariableFromSelectedVariableHolder()
   {
      YoVariable<?> yoVariable = selectedVariableHolder.getSelectedVariable();
      if (yoVariable != null)
         addVariable(dataEntryHolder.getEntry(yoVariable));
   }

   public void removeEntry(DataEntry entry)
   {
      if (entriesOnThisGraph.contains(entry))
         entriesOnThisGraph.remove(entry);

      // this.repaint();
      // Do updateUI instead of just repaint whenever adding or deleting stuff.

      /*
       * if (this.getNumVars() < 1) { Dimension dimension = getSize();
       * dimension.height = EMPTY_HEIGHT; this.setSize(dimension); }
       */

      this.reCalcMinMax();
      calculateRequiredEntryPaintWidthsAndRows();

      this.updateUI();
   }

   private boolean minMaxChanged()
   {
      // Returns true if the min,max changed on any data in this graph.
      boolean ret = false;

      int numVars = entriesOnThisGraph.size();
      if (numVars < 1)
         return ret;

      for (int i = 0; i < numVars; i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);
         ret = (ret || entry.minMaxChanged());
      }

      return ret;
   }

   private void reCalcMinMax()
   {
      int numVars = entriesOnThisGraph.size();
      if (numVars < 1)
         return;

      double newMin = Double.POSITIVE_INFINITY, newMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numVars; i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);
         boolean inverted = entry.getInverted();
         
         // entry.reCalcMinMax();
         // entry.resetMinMaxChanged();
         
         double entryMin = entry.getMin();
         double entryMax = entry.getMax();
         
         if (inverted)
         {
            double temp = entryMax;
            entryMax = -entryMin;
            entryMin = -temp;
         }
         
         if (entryMax > newMax)
            newMax = entryMax;
         if (entryMin < newMin)
            newMin = entryMin;
      }

      this.min = newMin;
      this.max = newMax;
   }

   private void resetMinMax()
   {
      int numVars = entriesOnThisGraph.size();
      if (numVars < 1)
         return;

      for (int i = 0; i < numVars; i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);
         entry.resetMinMaxChanged();
      }
   }

   private void calcXYData(DataEntry entry, int nPoints, int[] xData, int[] yData, double min, double max, int width, int height, int offsetFromLeft,
                           int offsetFromTop, int leftPlotIndex, int rightPlotIndex)
   {
      double[] data = entry.getData();

      boolean inverted = entry.getInverted();
      
      if (leftPlotIndex == rightPlotIndex)
      {
         for (int i = 0; i < nPoints; i++)
         {
            xData[i] = offsetFromLeft;
            yData[i] = offsetFromTop;
         }
      }

      else
      {
         for (int i = 0; i < nPoints; i++)
         {
            double dataAtTick = data[i];
            
            if (inverted) dataAtTick = -dataAtTick;
            xData[i] = ((i - leftPlotIndex) * width) / (rightPlotIndex - leftPlotIndex) + offsetFromLeft;
            yData[i] = height - (int) ((dataAtTick - min) / (max - min) * height) + offsetFromTop;
         }
      }
   }

   private void calcScatterData(DataEntry entryX, DataEntry entryY, int nPoints, int[] xData, int[] yData, double minX, double maxX, double minY, double maxY,
                                int width, int height, int offsetFromLeft, int offsetFromTop)
   {
      double[] dataX = entryX.getData();
      double[] dataY = entryY.getData();

      for (int i = 0; i < nPoints; i++)
      {
         // xData[i] = width - (int) ((dataX[i] - minX)/(maxX-minX) * width) + offsetFromLeft;
         xData[i] = (int) ((dataX[i] - minX) / (maxX - minX) * width) + offsetFromLeft;
         yData[i] = height - (int) ((dataY[i] - minY) / (maxY - minY) * height) + offsetFromTop;
      }
   }

   protected synchronized void printGraph(Graphics g, int printWidth, int printHeight)
   {
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      int numVars = entriesOnThisGraph.size();
      if (numVars == 0)
      {
         return;
      }

      int cumOffset = 3;

      // Here we use one scale for all the graphs:
      this.reCalcMinMax();

      for (int i = 0; i < numVars; i++)
      {
         cumOffset = i * ((int) (VAR_NAME_SPACING_FOR_PRINT * 0.6)) + 3;

         DataEntry entry = entriesOnThisGraph.get(i);
         double[] data = entry.getData();

         double minVal = 0.0, maxVal = 1.0;
         if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING)
         {
            if (entry.isAutoScaleEnabled())
            {
               minVal = entry.getMin();
               maxVal = entry.getMax();
            }
            else
            {
               minVal = entry.getManualMinScaling();
               maxVal = entry.getManualMaxScaling();
            }
         }

         else if (graphConfiguration.getScalingMethod() == AUTO_SCALING)
         {
            minVal = this.min;
            maxVal = this.max;
         }

         else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING)
         {
            minVal = graphConfiguration.getManualScalingMin();
            maxVal = graphConfiguration.getManualScalingMax();
         }

         int nPoints = data.length;

         int length = ((outPoint - inPoint + 1 + nPoints) % nPoints);
         if (length == 0)
            length = nPoints;

         // System.out.println("length:  " + length);

         int[] xDataPrint = new int[length];
         int[] yDataPrint = new int[length];

         for (int j = 0; j < length; j++)
         {
            int index = (inPoint + j) % nPoints;
            xDataPrint[j] = (j * printWidth) / length;
            yDataPrint[j] = (printHeight - DONT_PLOT_BOTTOM_PIXELS)
                            - (int) ((data[index] - minVal) / (maxVal - minVal) * (printHeight - DONT_PLOT_BOTTOM_PIXELS));
         }

         g.setColor(colors[i % YoGraph.MAX_NUM_GRAPHS]);

         // Draw the data
         g.drawPolyline(xDataPrint, yDataPrint, xDataPrint.length);

         // Draw the variable name
         String dString = entry.getVariableName();
         g.drawString(dString, cumOffset, printHeight);

      }
   }

   private static final String clickMessage = new String("Middle Click to graph selected variable");

   public void repaintAllGraph()
   {
      // if((scaling == AUTO_SCALING) && this.minMaxChanged())
      if (this.minMaxChanged())
      {
         if (graphConfiguration.getScalingMethod() == AUTO_SCALING)
            this.reCalcMinMax();
      }

      repaint();
   }

   public void repaintPartialGraph(int index, int oldIndex, int inPoint, int outPoint, int leftPlotIndex, int rightPlotIndex)
   {
      if (leftPlotIndex == rightPlotIndex)
         return;

      if (this.minMaxChanged())
      {
         if (graphConfiguration.getScalingMethod() == AUTO_SCALING)
         {
            this.reCalcMinMax();
         }
         else
         {
            resetMinMax();
         }

         repaint();

         return;
      }

      int width = this.getWidth() - 6;
      int offsetFromLeft = 3;

      int xStart = ((oldIndex - leftPlotIndex) * width) / (rightPlotIndex - leftPlotIndex) + offsetFromLeft - 1;
      int xEnd = ((index - leftPlotIndex) * width) / (rightPlotIndex - leftPlotIndex) + offsetFromLeft;

      if (xStart < 0)
         xStart = 0;
      if (xEnd < 0)
         xEnd = 0;

      if (index > oldIndex)
      {
         // Repaint the portion of the graph that changed:
         repaint(xStart, 0, xEnd - xStart + 1, getHeight());

         // Repaint the region where the nueric values are:
         repaint(0, getHeight() - 20, getWidth() - 1, 20);
      }

      // else repaint();
   }

   public void repaintGraphOnSetPoint(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex)
   {
      if (leftIndex == rightIndex)
      {
         return;
      }

      double newMin = Double.POSITIVE_INFINITY;
      double newMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < entriesOnThisGraph.size(); i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);
         boolean inverted = entry.getInverted();

         double entryMax = entry.getMax(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);
         double entryMin = entry.getMin(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);
         
         if (inverted)
         {
            double temp = entryMax;
            entryMax = - entryMin;
            entryMin = -temp;
         }
         
         if (entryMax > newMax)
         {
            newMax = entryMax;
         }

         if (entryMin < newMin)
         {
            newMin = entryMin;
         }
      }

      this.min = newMin;
      this.max = newMax;

   }

   private StringBuffer stringBuffer = new StringBuffer(80);
   @SuppressWarnings("unused")
   private String spaceString = "  ";
   private char[] charArray = new char[80];
   private final java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");
   private final FieldPosition fieldPosition = new FieldPosition(NumberFormat.INTEGER_FIELD);
   
   public void createBodePlotFromEntriesBetweenInOutPoints()
   {
	    if (entriesOnThisGraph.size() < 2){	     
	    	System.out.println("need 2 entries (input/output) for Bode plot");
	    	return;
	    }
	    if (!checkInOutPoints()) 
	         return;

	    int inPoint = graphIndicesHolder.getInPoint();
	    int outPoint = graphIndicesHolder.getOutPoint();
	    
	   DataEntry input = entriesOnThisGraph.get(0);
	   DataEntry output = entriesOnThisGraph.get(1);
	    
	    double[] inputData = Arrays.copyOfRange(input.getData(), inPoint, outPoint);
	    double[] outputData = Arrays.copyOfRange(output.getData(), inPoint, outPoint);

	    double[] timeData = Arrays.copyOfRange(timeDataHolder.getTimeData(), inPoint, outPoint);
	    
	    BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
   }

   public void createBodePlotFromEntries()
   {
      if (entriesOnThisGraph.size() < 2)
         return;

      DataEntry input = entriesOnThisGraph.get(0);
      DataEntry output = entriesOnThisGraph.get(1);

      double[] inputData = input.getData();
      double[] outputData = output.getData();

      double[] timeData = timeDataHolder.getTimeData();

      BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
   }

   private boolean checkInOutPoints()
   {
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      boolean valid = outPoint > inPoint;
      if(!valid)
      {
         System.out.println("Please set inPoint < outPoint and re-try");
      }
      return valid;   
   }
   
   public void createFFTPlotsFromEntriesBetweenInOutPoints()
   {
      if (!checkInOutPoints()) return;
    
      
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();
      

	  double[] timeData = timeDataHolder.getTimeData();	  
      double[] rngTimeData = Arrays.copyOfRange(timeData, inPoint, outPoint);

      for (DataEntry entry : entriesOnThisGraph)
      {
         double[] data = entry.getData();
         double[] rngData = Arrays.copyOfRange(data, inPoint, outPoint);

         BodePlotConstructor.plotFFT(entry.getVariableName(), rngTimeData, rngData);
      }

   }

   
   public void createFFTPlotsFromEntries()
   {

	  
      double[] timeData = timeDataHolder.getTimeData();

      for (DataEntry entry : entriesOnThisGraph)
      {
         double[] data = entry.getData();

         BodePlotConstructor.plotFFT(entry.getVariableName(), timeData, data);
      }

   }

   @Override
   public void paintComponent(Graphics g)
   {
      if (graphConfiguration.getPlotType() == YoGraph.TIME_PLOT)
         paintTimePlot(g);

         // else if (plotType == SCATTER_PLOT) paintScatterPlot(g);
      else if (graphConfiguration.getPlotType() == PHASE_PLOT)
         paintPhasePlot(g);
   }

   public void paintPhasePlot(Graphics g)
   {
      super.paintComponent(g);

      int graphWidth = this.getWidth();
      int graphHeight = this.getHeight();

      int numVars = entriesOnThisGraph.size();
      FontMetrics fontMetrics = this.getFontMetrics(getFont());

      // if (numVars < 2) {this.paintTimePlot(g); return;}

      if (numVars == 0)
      {
         g.setColor(Color.white);

         int messageWidth = fontMetrics.stringWidth(clickMessage);
         g.drawString(clickMessage, (graphWidth - messageWidth) / 2, graphHeight / 2);
      }

      for (int i = 0; i < entriesOnThisGraph.size() / 2; i++)
      {
         DataEntry entryX = entriesOnThisGraph.get(i);
         double[] dataX = entryX.getData();

         DataEntry entryY = entriesOnThisGraph.get(i + 1);

//       double[] dataY = entryY.getData();

         double minValX = 0.0, maxValX = 1.0;
         double minValY = 0.0, maxValY = 1.0;

         if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING)
         {
            if (entryX.isAutoScaleEnabled())
            {
               minValX = entryX.getMin();
               maxValX = entryX.getMax();
            }
            else
            {
               minValX = entryX.getManualMinScaling();
               maxValX = entryX.getManualMaxScaling();
            }

            if (entryY.isAutoScaleEnabled())
            {
               minValY = entryY.getMin();
               maxValY = entryY.getMax();
            }
            else
            {
               minValY = entryY.getManualMinScaling();
               maxValY = entryY.getManualMaxScaling();
            }
         }

         else if (graphConfiguration.getScalingMethod() == AUTO_SCALING)
         {
            // minValX = minValY = this.min;   //++++++
            // maxValX = maxValY = this.max;

            minValY = entryY.getMin();
            maxValY = entryY.getMax();

            minValX = entryX.getMin();
            maxValX = entryX.getMax();
         }

         else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING)
         {
            minValX = minValY = graphConfiguration.getManualScalingMin();    // ++++++
            maxValX = maxValY = graphConfiguration.getManualScalingMax();
         }

         int nPoints = dataX.length;
         if ((xData.length != nPoints) || (yData.length != nPoints))
         {
            xData = new int[nPoints];
            yData = new int[nPoints];
         }

         int totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);

         calcScatterData(entryX, entryY, nPoints, xData, yData, minValX, maxValX, minValY, maxValY, (graphWidth - 6), graphHeight - totalDontPlotBottomPixels,
                         3, 5);

         g.setColor(colors[i % YoGraph.MAX_NUM_GRAPHS]);

         // Draw the data
         g.drawPolyline(xData, yData, xData.length);

         // Draw a Cross Hairs:
         int index = graphIndicesHolder.getIndex();

         if ((index < xData.length) && (index < yData.length) & (index >= 0))
         {
            g.setColor(Color.BLACK);

            g.drawLine(xData[index] - 5, yData[index], xData[index] + 5, yData[index]);
            g.drawLine(xData[index], yData[index] - 10, xData[index], yData[index] + 10);
         }
      }

      paintVariableNamesAndValues(g, true);
   }

   private final Stroke dashedStroke = new BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0.0f, new float[]{9}, 0);
   private final Stroke wideStroke = new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0.0f);
   private final Stroke normalStroke = new BasicStroke();

   public void paintTimePlot(Graphics graphics)
   {
      super.paintComponent(graphics);
      Graphics2D g2d = (Graphics2D) graphics;

      int graphWidth = this.getWidth();
      int graphHeight = this.getHeight();

      if (graphWidth != previousGraphWidth)
      {
         calculateRequiredEntryPaintWidthsAndRows();
      }

      FontMetrics fontMetrics = this.getFontMetrics(getFont());

      int index = graphIndicesHolder.getIndex();
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      int leftPlotIndex = graphIndicesHolder.getLeftPlotIndex();
      int rightPlotIndex = graphIndicesHolder.getRightPlotIndex();

      int numVars = entriesOnThisGraph.size();

      if (numVars == 0)
      {
         graphics.setColor(Color.white);

         int messageWidth = fontMetrics.stringWidth(clickMessage);
         graphics.drawString(clickMessage, (graphWidth - messageWidth) / 2, graphHeight / 2);
      }

      // int cumulativeOffset = 3;
      // int previousRow = 0;

      for (int i = 0; i < numVars; i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);
         double[] data = entry.getData();

         double minVal = 0.0, maxVal = 1.0;
         if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING)
         {
            if (entry.isAutoScaleEnabled())
            {
               minVal = entry.getMin();
               maxVal = entry.getMax();
            }
            else
            {
               minVal = entry.getManualMinScaling();
               maxVal = entry.getManualMaxScaling();
            }
         }

         else if (graphConfiguration.getScalingMethod() == AUTO_SCALING)
         {
            minVal = this.min;
            maxVal = this.max;
         }

         else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING)
         {
            minVal = graphConfiguration.getManualScalingMin();
            maxVal = graphConfiguration.getManualScalingMax();
         }

         int nPoints = data.length;
         if ((xData.length != nPoints) || (yData.length != nPoints))
         {
            // System.out.println("Making new xData, yData!!!");
            xData = new int[nPoints];
            yData = new int[nPoints];
         }

         int totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);

         calcXYData(entry, nPoints, xData, yData, minVal, maxVal, (graphWidth - 6), graphHeight - totalDontPlotBottomPixels, 3, 5, leftPlotIndex,
                    rightPlotIndex);

         graphics.setColor(colors[i % YoGraph.MAX_NUM_GRAPHS]);

         // Draw the data
         g2d.setStroke(normalStroke);
         graphics.drawPolyline(xData, yData, xData.length);

         if (graphConfiguration.getShowBaseLines())
         {
            double[] baseLines = graphConfiguration.getBaseLines();

            for (int j=0; j<baseLines.length; j++)
            {
               double baseLine = baseLines[j];
               int baseY = (graphHeight - totalDontPlotBottomPixels)
                           - (int) ((baseLine - minVal) / (maxVal - minVal) * (graphHeight - totalDontPlotBottomPixels)) + 5;
                            
               graphics.setColor(baseLineColors[j]);

               // int baseY = (int) ( (baseLine - minVal) / (maxVal - minVal) * this.getHeight());;
               g2d.setStroke(dashedStroke);
               graphics.drawLine(0, baseY, this.getWidth(), baseY);
               g2d.setStroke(normalStroke);
            }
         }
      }

      // Paint vertical index line (but only once):

      paintVerticalIndexLines(graphics, graphWidth, graphHeight, index, inPoint, outPoint, leftPlotIndex, rightPlotIndex);
      paintVerticalTimeGrids(graphics, graphWidth,graphHeight);
      paintVariableNamesAndValues(graphics, false);
   }
   
   

   private void paintVerticalTimeGrids(Graphics graphics, int graphWidth, int graphHeight)
   {
      Graphics2D g2d = (Graphics2D) graphics;
      graphics.setColor(Color.green);
      g2d.setStroke(wideStroke);
      
//      double[] times = this.timeDataHolder.getTimeData();
      //graphics.drawLine(linex, 0, linex, graphTopYValue);

      graphics.setColor(Color.red);
      g2d.setStroke(normalStroke);
   }


   private void paintVerticalIndexLines(Graphics graphics, int graphWidth, int graphHeight, int index, int inPoint, int outPoint, int leftPlotIndex,
           int rightPlotIndex)
   {
      Graphics2D g2d = (Graphics2D) graphics;

      graphics.setColor(Color.green);
      int leftToRightPlotIndexDelta = rightPlotIndex - leftPlotIndex;
      if (leftToRightPlotIndexDelta == 0)
         return;

      int linex = ((graphWidth - 6) * (inPoint - leftPlotIndex)) / leftToRightPlotIndexDelta + 3;

      int totalDontPlotTimelineBottomPixels = DONT_PLOT_TIMELINE_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);
      int graphTopYValue = graphHeight - totalDontPlotTimelineBottomPixels;

      g2d.setStroke(wideStroke);

      graphics.drawLine(linex, 0, linex, graphTopYValue);

      graphics.setColor(Color.red);
      linex = ((graphWidth - 6) * (outPoint - leftPlotIndex)) / leftToRightPlotIndexDelta + 3;
      graphics.drawLine(linex, 0, linex, graphTopYValue);

      graphics.setColor(Color.ORANGE);

      ArrayList<Integer> keys = graphIndicesHolder.getKeyPoints();

      for (int i = 0; i < keys.size(); i++)
      {
         linex = ((graphWidth - 6) * (keys.get(i) - leftPlotIndex)) / leftToRightPlotIndexDelta + 3;
         graphics.drawLine(linex, 0, linex, graphTopYValue);
      }

      graphics.setColor(Color.black);

      linex = ((graphWidth - 6) * (index - leftPlotIndex)) / leftToRightPlotIndexDelta + 3;
      graphics.drawLine(linex, 0, linex, graphTopYValue);
      
      g2d.setStroke(normalStroke);
   }

   private void paintVariableNamesAndValues(Graphics g, boolean phasePlot)
   {
      int previousRow = 0;
      int cumulativeOffset = 3;
      
      if (showBaseLines)
      {
         drawBaseLines(g);
         return;
      }
      
      for (int i = 0; i < entriesOnThisGraph.size(); i++)
      {
         DataEntry entry = entriesOnThisGraph.get(i);

         if (phasePlot)
            g.setColor(colors[i / 2 % YoGraph.MAX_NUM_GRAPHS]);
         else
            g.setColor(colors[i % YoGraph.MAX_NUM_GRAPHS]);

         // Draw the variable name
         int row = entryNamePaintRows.get(i);
         if (row != previousRow)
         {
            cumulativeOffset = 3;
            previousRow = row;
         }

         drawVariableNameAndValue(g, cumulativeOffset, row, entry);
         cumulativeOffset += entryNamePaintWidths.get(i);
      }
   }

   private void drawBaseLines(Graphics g)
   {
      if (!graphConfiguration.getShowBaseLines()) return;
      
      double[] baseLines = graphConfiguration.getBaseLines();
      if (baseLines.length == 0) return;
      
      int row = 0;
      int graphHeight = this.getHeight();
      int yToDrawAt = graphHeight - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - row - 1));
      
      int cumOffset = 3;
   
      double total = 0.0;

      g.setColor(Color.black);
      g.drawString("BaseLines: ", cumOffset, yToDrawAt);
      cumOffset += 80;
      
      for (int i=0; i<baseLines.length; i++)
      {
         stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...
         double baseLine = baseLines[i];
         total = total + baseLine;
         
         formatDouble(stringBuffer, baseLine);
         String baseLineString = stringBuffer.toString();

         FontMetrics fontMetrics = this.getFontMetrics(getFont());
         int baseLineStringWidth = fontMetrics.stringWidth(baseLineString);
         
         g.setColor(colors [i]);
         g.drawString(baseLineString, cumOffset, yToDrawAt);
         cumOffset = cumOffset + baseLineStringWidth + 10;
      }

      double average = total / ((double) baseLines.length);
      
      g.setColor(Color.black);
      
      stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...
      formatDouble(stringBuffer, average);
      String averageString = stringBuffer.toString();

      g.drawString("    Average = " + averageString, cumOffset, yToDrawAt);    
   }
   
   
   private void drawVariableNameAndValue(Graphics g, int cumOffset, int row, DataEntry entry)
   {
      int graphHeight = this.getHeight();

      stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...

      // +++JEP 12/3/2012: Draw the value at the index, not the current YoVariable value. That way if a robot thread is updating
      // Sensor information, you see the saved value, not the value as it's being updated.

      if (graphIndicesHolder.isIndexAtOutPoint())
      {
         entry.getVariableNameAndValue(stringBuffer);
      }
      else
      {
         entry.getVariableNameAndValueAtIndex(stringBuffer, graphIndicesHolder.getIndex());
      }
      
      if(showNameSpace)
      {
         NameSpace nameSpace = entry.getVariable().getNameSpace();
         stringBuffer.insert(0, nameSpace);
      }

      int length = Math.min(stringBuffer.length(), charArray.length);
      stringBuffer.getChars(0, length, charArray, 0);    // dump it into the character Array

      int yToDrawAt = graphHeight - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - row - 1));
      g.drawChars(charArray, 0, length, cumOffset, yToDrawAt);    // Print it.
   }

   @Override
   public void keyPressed(KeyEvent evt)
   {
      int code = evt.getKeyCode();

      switch(code)
      {
      case KeyEvent.VK_LEFT:
         this.graphIndicesHolder.tickLater(-1);
         break;
      case KeyEvent.VK_RIGHT:
         this.graphIndicesHolder.tickLater(1);
         break;
      case KeyEvent.VK_ALT:
         showNameSpace = true;
         calculateRequiredEntryPaintWidthsAndRows();
         repaint();
         break;
      case KeyEvent.VK_CONTROL:
         showBaseLines = true;
         calculateRequiredEntryPaintWidthsAndRows();
         repaint();
         break;
      case KeyEvent.VK_UP:
         incrementBaseLine(focussedBaseLine, 1.0);
         repaint();
         break;
      case KeyEvent.VK_DOWN:
         incrementBaseLine(focussedBaseLine, -1.0);
         repaint();
         break;
      }
   }

   @Override
   public void keyReleased(KeyEvent evt)
   {
      int code = evt.getKeyCode();

      switch(code)
      {
      case KeyEvent.VK_ALT:
         showNameSpace = false;
         calculateRequiredEntryPaintWidthsAndRows();
         repaint();
         break;

      case KeyEvent.VK_CONTROL:
         showBaseLines = false;
         calculateRequiredEntryPaintWidthsAndRows();
         repaint();
         break;
      }
   }

   @Override
   public void keyTyped(KeyEvent evt)
   {
      char character = evt.getKeyChar();

      switch(character)
      {
      case '1':
         this.focussedBaseLine = 0;
         repaint();
         break;
      case '2':
         this.focussedBaseLine = 1;
         repaint();
         break;
      case '3':
         this.focussedBaseLine = 2;
         repaint();
         break;
      case '4':
         this.focussedBaseLine = 3;
         repaint();
         break;
      case '5':
         this.focussedBaseLine = 4;
         repaint();
         break;
      case '6':
         this.focussedBaseLine = 5;
         repaint();
         break;
      case 'z':
         zeroBaseLine(focussedBaseLine);
         repaint();
         break;
      case 'c':
         centerBaseLine(focussedBaseLine);
         repaint();
         break;
      }

   }

   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }

   @Override
   public void mousePressed(MouseEvent evt)
   {
      if (evt.getSource().equals(this))
      {
         popupMenu.setVisible(false);
      }

      this.requestFocus();
      int y = evt.getY();
      int x = evt.getX();
      int h = getHeight();
      int w = getWidth();

      // Remember stuff for drag...
      this.clickedX = x;
      this.clickedY = y;
      this.draggedX = x;
      this.draggedY = y;
      this.clickedIndex = clickIndex(x, w);
      this.clickedLeftIndex = graphIndicesHolder.getLeftPlotIndex();
      this.clickedRightIndex = graphIndicesHolder.getRightPlotIndex();

      // Double click brings up var properties dialog box.
      if ((evt.getClickCount() == 2) && (!entriesOnThisGraph.isEmpty()))
      {
         // System.out.println("Double Clicked!!!");
         if (parentFrame != null)
         {
            GraphPropertiesDialog dialog = new GraphPropertiesDialog(parentFrame, this);
            dialog.pack();
            dialog.setVisible(true);

            // parentFrame.repaint(); // This is a horrible way to get the graphs to repaint...
         }

      }

      // Right click places and deletes graphs:

      // if (evt.isControlDown())
      // if (evt.isMetaDown() && evt.isAltDown())
      // if (evt.isMetaDown() && evt.isControlDown())
      // if (evt.isShiftDown())
      if (!(evt.isMetaDown()) && (evt.isAltDown()))
      {    // Middle Click
         // If mouse was pressed in a label, remove that variable:

         if (y > h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)
         {
            int idx = getClickedVariableIndex(x, y, h);

            if (idx < this.entriesOnThisGraph.size())
            {
               // System.out.println("Removing Variable from graph!");
               this.removeEntry(this.entriesOnThisGraph.get(idx));
            }
         }

         else
         {
            addVariableFromSelectedVariableHolder();
         }
      }

      // Left click places index:

      else if (!evt.isMetaDown() &&!evt.isAltDown() &&!GUISideCommandListener.isRecording())
      {
         if ((this.entriesOnThisGraph == null) || (this.entriesOnThisGraph.size() < 1) || (getPlotType() == PHASE_PLOT))
         {
            return;
         }

         if (y <= h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)
         {
            int newIndex = clickIndex(x, w);

            // if (nPoints > 0)
            this.graphIndicesHolder.setIndexLater(newIndex);

            // this.graphArrayPanel.setIndexLater((x*nPoints)/w);
         }

         // If mouse was pressed in a label, highlight that variable and initiate drag and drop:
         else
         {
            int index = getClickedVariableIndex(x, y, h);

            if (index < this.entriesOnThisGraph.size())
            {
               DataEntry entry = entriesOnThisGraph.get(index);
               selectedVariableHolder.setSelectedVariable(entry.getVariable());

               if (!evt.isControlDown())
               {
                  this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.MOVE);
                  actionPerformedByDragAndDrop = TransferHandler.MOVE;
               }
               else if (evt.isControlDown())
               {
                  this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.COPY);
                  actionPerformedByDragAndDrop = TransferHandler.COPY;
               }

               sourceOfDrag = this;

               repaint();
            }
         }

      }
      else if (evt.isMetaDown() &&!evt.isAltDown())
      {
         popupMenu.remove(delete);
         Component[] components = popupMenu.getComponents();
         for (Component component : components)
         {
            popupMenu.remove(component);
         }

         for (final DataEntry dataBufferEntry : entriesOnThisGraph)
         {
            final JMenuItem menuItem = new JMenuItem("Remove " + dataBufferEntry.getVariableName());
            menuItem.addActionListener(new ActionListener()
            {
               @Override
               public void actionPerformed(ActionEvent e)
               {
                  removeEntry(dataBufferEntry);
                  popupMenu.remove(menuItem);
                  popupMenu.setVisible(false);
                  popupMenu.invalidate();
                  popupMenu.revalidate();
               }
            });
            popupMenu.add(menuItem);
         }

         popupMenu.add(delete);
         popupMenu.setLocation(evt.getXOnScreen(), evt.getYOnScreen());
         popupMenu.setVisible(true);
      }

   }

   private int getClickedVariableIndex(int x, int y, int graphHeight)
   {
      int rowClicked = (y - (graphHeight - totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)) / PIXELS_PER_BOTTOM_ROW;

      // System.out.println("rowClicked = " + rowClicked);

      int index = 0;
      int totalOffset = 3;

      for (int j = 0; j < entryNamePaintWidths.size(); j++)
      {
         Integer entryPaintWidth = entryNamePaintWidths.get(j);
         Integer row = entryNamePaintRows.get(j);

         if (row < rowClicked)
         {
            index++;
         }
         else
         {
            totalOffset = totalOffset + entryPaintWidth;
            if (x > totalOffset)
               index++;
         }
      }

      return index;
   }

   private int clickIndex(int x, int w)
   {
      int leftPlotIndex = graphIndicesHolder.getLeftPlotIndex();
      int rightPlotIndex = graphIndicesHolder.getRightPlotIndex();

      return clickIndex(x, w, leftPlotIndex, rightPlotIndex);    // (leftPlotIndex + (2*x*(rightPlotIndex-leftPlotIndex)+w)/(2*w));
   }

   private int clickIndex(int x, int w, int leftPlotIndex, int rightPlotIndex)
   {
      return (leftPlotIndex + (2 * x * (rightPlotIndex - leftPlotIndex) + w) / (2 * w));
   }

   @Override
   public void mouseMoved(MouseEvent evt)
   {
   }

   @SuppressWarnings("unused")
   private int clickedX, clickedY;
   @SuppressWarnings("unused")
   private int draggedX, draggedY;
   private int clickedIndex, clickedLeftIndex, clickedRightIndex;

   @Override
   public void mouseClicked(MouseEvent evt)
   {
   }

   @Override
   public void mouseDragged(MouseEvent evt)
   {
      draggedX = evt.getX();
      draggedY = evt.getY();

      int h = getHeight();
      int w = getWidth();

      if (draggedX > w)
         draggedX = w;
      if (draggedX < 0)
         draggedX = 0;

      if (clickedY > h - DONT_PLOT_TIMELINE_BOTTOM_PIXELS)
         return;

      if (!evt.isMetaDown() && (!evt.isAltDown()) && (getPlotType() != PHASE_PLOT) &&!GUISideCommandListener.isRecording() )
      {    // Left Click n Drag
         int index = clickIndex(draggedX, w, clickedLeftIndex, clickedRightIndex);
         graphIndicesHolder.setIndexLater(index);    // +++JEP setIndex or setIndexLater??

         // graphArrayPanel.setIndex(index); //+++JEP setIndex or setIndexLater??
         // graphArrayPanel.repaintGraphs();
         // clickIndex = index;

         // System.out.println("x: " + draggedX + "index: " + index);
      }

      if (evt.isMetaDown() && (!evt.isAltDown()))
      {    // Right Click n Drag
         // draggedX = evt.getX();

         // if (draggedX > w) draggedX = w;
         // if (draggedX < 0) draggedX = 0;

         // System.out.println(draggedX);
         int index = clickIndex(draggedX, w, clickedLeftIndex, clickedRightIndex);

         // System.out.println("Mouse Dragged!!");

         int newLeftIndex = clickedLeftIndex + clickedIndex - index;
         int newRightIndex = clickedRightIndex + clickedIndex - index;

         if (newLeftIndex < 0)
         {
            newLeftIndex = 0;
            newRightIndex = clickedRightIndex - clickedLeftIndex;
         }

         if (newRightIndex > graphIndicesHolder.getMaxIndex())
         {
            newRightIndex = graphIndicesHolder.getMaxIndex();
            newLeftIndex = newRightIndex - (clickedRightIndex - clickedLeftIndex);
         }

         // System.out.println("x: " + draggedX + "newLeft: " + newLeftIndex + "newRight: " + newRightIndex);

         graphIndicesHolder.setLeftPlotIndex(newLeftIndex);
         graphIndicesHolder.setRightPlotIndex(newRightIndex);

         // graphArrayPanel.repaintGraphs();
      }
   }

// public DataBuffer getDataBuffer()
// {
//    return dataBuffer;
// }
//
// public SelectedVariableHolder getSelectedVariableHolder()
// {
//    return selectedVariableHolder;
// }

   @Override
   public void focusGained(FocusEvent arg0)
   {
      if (arg0.getSource().equals(this) && hadFocus)
      {
         popupMenu.setVisible(false);
      }

      hadFocus = true;
   }

   @Override
   public void focusLost(FocusEvent arg0)
   {
      popupMenu.setVisible(false);
      hadFocus = false;
   }

   public static int getActionPerformedByDragAndDrop()
   {
      return actionPerformedByDragAndDrop;
   }

   public static void setActionPerformedByDragAndDrop(int actionPerformedByDragAndDrop)
   {
      YoGraph.actionPerformedByDragAndDrop = actionPerformedByDragAndDrop;
   }

   public static Object getSourceOfDrag()
   {
      return sourceOfDrag;
   }

   public static void setSourceOfDrag(Object sourceOfDrag)
   {
      YoGraph.sourceOfDrag = sourceOfDrag;
   }

   public static Object getRecipientOfDrag()
   {
      return recipientOfDrag;
   }

   public static void setRecipientOfDrag(Object recipientOfDrag)
   {
      YoGraph.recipientOfDrag = recipientOfDrag;
   }


   private void formatDouble(StringBuffer stringBuffer, double doubleValue)
   {
      doubleFormat.format(doubleValue, stringBuffer, fieldPosition); // Add the variable value to it
   }

}
