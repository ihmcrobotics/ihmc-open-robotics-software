package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.print.PageFormat;
import java.awt.print.Printable;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFX3DGraph;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;

public class GraphArrayPanel extends JPanel implements GraphIndicesHolder, YoGraphRemover, DataBufferChangeListener, Printable, ZoomGraphCommandExecutor
{
   private static final long serialVersionUID = -4366771635271760899L;

   private ArrayList<YoGraph> graphsOnThisPanel;
   private ArrayList<JavaFX3DGraph> javaFX3DGraphs;

   private StandardSimulationGUI standardSimulationGUI;
   private JFrame parentFrame;
   private DataBuffer dataBuffer;

   private int numColumns = 1;
   public final int MAX_GRAPHS = 24;
   public final int MAX_COLS = 4;

   private int leftPlotIndex;
   private int rightPlotIndex;

   private SelectedVariableHolder selectedVariableHolder;

   public GraphArrayPanel(SelectedVariableHolder holder, DataBuffer buffer, JFrame frame, StandardSimulationGUI standardSimulationGUI)
   {
      // super(new GridLayout(0,2,2,2));
      super(new GridLayout(0, 1, 2, 2));
      this.selectedVariableHolder = holder;
      this.setBackground(Color.lightGray);

      this.standardSimulationGUI = standardSimulationGUI;
      this.parentFrame = frame;
      this.dataBuffer = buffer;

      leftPlotIndex = 0;
      rightPlotIndex = getMaxIndex();

      this.setOpaque(true);
      this.graphsOnThisPanel = new ArrayList<>(16);
      this.javaFX3DGraphs = new ArrayList<>();

      this.setPreferredSize(new Dimension(800, 400));
   }

   public int getNumberOfColumns()
   {
      return numColumns;
   }

   public ArrayList<YoGraph> getGraphsOnThisPanel()
   {
      return graphsOnThisPanel;
   }

   public void setNumColumns(int numColumns)
   {
      this.numColumns = numColumns;
      this.setLayout(new GridLayout(0, numColumns, 2, 2));
      updateGraphs(); 
   }

   public void addColumn()
   {
      if (numColumns >= this.MAX_COLS)
         return;
      numColumns++;
      this.setLayout(new GridLayout(0, numColumns, 2, 2));
      updateGraphs();
   }

   public void subColumn()
   {
      if (numColumns <= 1)
         return;
      this.numColumns--;
      this.setLayout(new GridLayout(0, numColumns, 2, 2));
      updateGraphs();
   }

   @Override
   public void dataBufferChanged()
   {
      this.zoomFullView();
   }

   public void setInteractionEnable(boolean enable)
   {
      for (int i = 0; i < this.graphsOnThisPanel.size(); i++)
      {
         YoGraph yoGraph = this.graphsOnThisPanel.get(i);
         yoGraph.setInteractionEnable(enable);
      }

   }

   private int oldIndex = 99;

   // private boolean repaintAll = true;
   public void repaintGraphs()
   {
      int index = this.getIndex();

      // if (index == oldIndex) return;

      int inPoint = this.getInPoint();
      int outPoint = this.getOutPoint();

      int leftPlotIndex = this.getLeftPlotIndex();
      int rightPlotIndex = this.getRightPlotIndex();

      boolean repaintAll = (index < oldIndex);

      if ((index < leftPlotIndex) || (index > rightPlotIndex))
      {
         this.recenter();
         leftPlotIndex = getLeftPlotIndex();
         rightPlotIndex = getRightPlotIndex();
         repaintAll = true;
      }

      for (int i = 0; i < this.graphsOnThisPanel.size(); i++)
      {
         YoGraph g = this.graphsOnThisPanel.get(i);
         if (g.getNumVars() > 0)
         {
            if (repaintAll)
               g.repaintAllGraph();
            else
               g.repaintPartialGraph(index, oldIndex, inPoint, outPoint, leftPlotIndex, rightPlotIndex);
         }
      }

      oldIndex = index;
   }

   private boolean isPainting = false;

   @Override
   public void paint(Graphics g)
   {
      isPainting = true;
      super.paint(g);
      isPainting = false;
   }

   public boolean isPaintingPanel()
   {
      return isPainting;
   }

   public void goToInPointNow()
   {
      dataBuffer.gotoInPoint();
   }

   public void goToOutPointNow()
   {
      dataBuffer.gotoOutPoint();
   }

   public boolean tick(int n)
   {
      boolean ret = dataBuffer.tick(n);
      this.repaintGraphs();

      // this.repaint(); //+++JEP
      return ret;
   }

   @Override
   public int getInPoint()
   {
      return dataBuffer.getInPoint();
   }

   @Override
   public int getOutPoint()
   {
      return dataBuffer.getOutPoint();
   }

   @Override
   public int getIndex()
   {
      return dataBuffer.getIndex();
   }

   @Override
   public boolean isIndexAtOutPoint()
   {
      return (getIndex() == getOutPoint());
   }

   @Override
   public int getMaxIndex()
   {
      return dataBuffer.getBufferSize() - 1;
   }

   @Override
   public int getLeftPlotIndex()
   {
      return this.leftPlotIndex;
   }

   @Override
   public int getRightPlotIndex()
   {
      return this.rightPlotIndex;
   }

   @Override
   public void setLeftPlotIndex(int idx)
   {
      this.leftPlotIndex = idx;
      repaintGraphs();
   } 

   @Override
   public void setRightPlotIndex(int idx)
   {
      this.rightPlotIndex = idx;
      repaintGraphs();
   } 

   public void zoomFullView()
   {
      rightPlotIndex = getMaxIndex();
      leftPlotIndex = 0;
      updateGraphs();
   }

   @Override
   public void zoomIn()
   {
      zoomIn(2);
   }

   public void zoomIn(int factor)
   {
      int index = this.getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;
      int newLength = oldLength / factor;

      if (newLength < 4)
         return;

      leftPlotIndex = index - newLength / 2; // index - (index-leftPlotIndex)/factor;
      rightPlotIndex = leftPlotIndex + newLength; // index + (rightPlotIndex - index)/factor;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;
         rightPlotIndex = leftPlotIndex + newLength;
         if (rightPlotIndex > getMaxIndex())
            rightPlotIndex = getMaxIndex();
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();
         leftPlotIndex = rightPlotIndex - newLength;
         if (leftPlotIndex < 0)
            leftPlotIndex = 0;
      }

      repaint();
   }

   @Override
   public void zoomOut()
   {
      zoomOut(2);
   }

   public void zoomOut(int factor)
   {
      int index = this.getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;
      int newLength = oldLength * factor;

      leftPlotIndex = index - newLength / 2; // index - (index-leftPlotIndex)*factor;
      rightPlotIndex = leftPlotIndex + newLength; // index + (rightPlotIndex - index)*factor;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;
         rightPlotIndex = leftPlotIndex + newLength;
         if (rightPlotIndex > getMaxIndex())
            rightPlotIndex = getMaxIndex();
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();
         leftPlotIndex = rightPlotIndex - newLength;
         if (leftPlotIndex < 0)
            leftPlotIndex = 0;
      }

      repaint();
   }

   public void recenter()
   {
      zoomIn(1);
   }

   private int doTick = 0;
   private int doIndex = -1;

   @Override
   public void tickLater(int n)
   {
      if (dataBuffer.isKeyPointModeToggled())
      {
         if (n > 0)
            setIndexLater(dataBuffer.getNextTime());
         else
            setIndexLater(dataBuffer.getPreviousTime());
      }
      else
         this.doTick = n;
   }

   @Override
   public void setIndexLater(int idx)
   {
      this.doIndex = idx;
   }

   public boolean allowTickUpdatesNow()
   {
      boolean ret = false;

      if (this.doTick != 0)
      {
         dataBuffer.tick(doTick);

         // this.repaintGraphs();
         ret = true;
         doTick = 0;
      }

      if (this.doIndex != -1)
      {
         dataBuffer.setIndex(this.doIndex);
         this.doIndex = -1;

         // this.repaintGraphs();
         ret = true;

         // this.repaint();
      }

      return ret;

   }

   public void setupGraph(String varname)
   {
      final DataBufferEntry entry = dataBuffer.getEntry(varname);

      if (entry != null)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               YoGraph g = new YoGraph(getGraphArrayPanel(), getGraphArrayPanel(), selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
               g.addVariable(entry);
               addGraph(g);
            }

         });
      }
   }

   private GraphArrayPanel getGraphArrayPanel()
   {
      return this;
   }

   public void setupGraph(final String[] varnames)
   {
      if (varnames == null)
         return;

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            YoGraph g = new YoGraph(getGraphArrayPanel(), getGraphArrayPanel(), selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);

            for (int i = 0; i < varnames.length; i++)
            {
               DataBufferEntry entry = dataBuffer.getEntry(varnames[i]);

               if (entry != null)
                  g.addVariable(entry);
            }

            addGraph(g);
         }

      });
   }

   public void setupGraph(String[] varnames, GraphConfiguration config)
   {
      if (varnames == null)
         return;

      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      for (int i = 0; i < varnames.length; i++)
      {
         DataBufferEntry entry = dataBuffer.getEntry(varnames[i]);
         if (entry != null)
            g.addVariable(entry);
      }

      if (config != null)
         g.setGraphConfiguration(config);
      addGraph(g);
   }

   public void RepaintOnSetPoint()
   {
      for (int i = 0; i < graphsOnThisPanel.size(); i++)
      {
         graphsOnThisPanel.get(i).repaintGraphOnSetPoint(this.getInPoint(), this.getOutPoint(), getLeftPlotIndex(), getRightPlotIndex());
      }
   }

   public void addSelectedVariableGraph()
   {
      YoVariable<?> variable = selectedVariableHolder.getSelectedVariable();
      DataBufferEntry entry = dataBuffer.getEntry(variable);
      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      g.addVariable(entry);
      addGraph(g);
   }

   public void addEmptyGraph()
   {
      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      addGraph(g);
   }

   public void addNew3dGraph()
   {
      JavaFX3DGraph javaFX3DGraph = new JavaFX3DGraph(this, selectedVariableHolder, dataBuffer, dataBuffer);
      javaFX3DGraphs.add(javaFX3DGraph);
      standardSimulationGUI.setupExtraPanels(new ExtraPanelConfiguration("3D Graph " + javaFX3DGraphs.size(),
                                                                         javaFX3DGraph.getPanel(),
                                                                         true));
      standardSimulationGUI.selectPanel("3D Graph " + javaFX3DGraphs.size());
   }

   public void removeEmptyGraphs()
   {
      YoGraph emptyGraph = null;

      // Get the last one that has zero elements that needs to be deleted

      for (int i = 0; i < graphsOnThisPanel.size(); i++)
      {
         YoGraph graph = graphsOnThisPanel.get(i);
         if (graph.getNumVars() == 0)
         {
            emptyGraph = graph;
         }
      }

      if (emptyGraph != null)
      {
         this.remove(emptyGraph);
         this.graphsOnThisPanel.remove(emptyGraph);
         removeEmptyGraphs();
      }

      else
      {
          this.updateGraphs();
      }
   }

   public void addGraph(YoGraph graph)
   {
      int numGraphs = graphsOnThisPanel.size();

      if (numGraphs >= this.MAX_GRAPHS)
         return;

      this.graphsOnThisPanel.add(graph);
      this.add(graph);

       this.updateGraphs();
   }

   @Override
   public int print(Graphics g, PageFormat pageFormat, int pageNumber)
   {
      Graphics2D g2 = (Graphics2D) g;

      // System.out.println("In GraphArrayPanel.print");
      if (pageNumber == 0)
      {
         // First clear the graphics...
         // g2.setColor(Color.white);

         // g2.clearRect((int)pageFormat.getImageableX(),(int)pageFormat.getImageableY(),(int)pageFormat.getImageableWidth(),(int)pageFormat.getImageableHeight());
         // g2.setColor(Color.black);

         g2.translate(pageFormat.getImageableX(), pageFormat.getImageableY());

         // Scale:
         double pageWidth = pageFormat.getImageableWidth();
         double pageHeight = pageFormat.getImageableHeight();

         YoGraph graph = this.graphsOnThisPanel.get(0);

         // double width = graph.getWidth();
         // double height = this.graphsOnThisPanel.size() * graph.getHeight() * 1.25;

         // double scaleX = pageWidth/width;
         // double scaleY = pageHeight/height;

         // double scaleFactor = Math.min(scaleX, scaleY);

         // g2.scale(scaleFactor, scaleFactor);

         for (int i = 0; i < this.graphsOnThisPanel.size(); i++)
         {
            graph = this.graphsOnThisPanel.get(i);

            if (graph.getEntriesOnThisGraph().size() > 0)
            {
               // graph.paint(g2);
               graph.printGraph(g2, (int) pageWidth, (int) (pageHeight / 10.0));

               // g2.translate(0.0, graph.getHeight()*1.25);
               g2.translate(0.0, pageHeight / 8.0);
            }
         }

         // this.paint(g2);

         // g2.drawRect(10,10,50,50);
         return Printable.PAGE_EXISTS;
      }

      return Printable.NO_SUCH_PAGE;
   }

   @Override
   public void paintComponent(Graphics g)
   {
      // System.out.println("Painting GraphArrayPanel");
      super.paintComponent(g);

      // this.paintChildren(g);

      // Repaint all the graphs.  I thought this should be done automatically!!!!
      // for (int i=0; i<this.graphsOnThisPanel.size();i++)
      // {
      // YoGraph child = (YoGraph) this.graphsOnThisPanel.get(i);

      // System.out.println("Requesting child " + i + " to repaint itself");
      // child.repaint();

      // }
   }

   public void closeAndDispose()
   {
      parentFrame = null;
      dataBuffer = null;

      if (graphsOnThisPanel != null)
      {
         graphsOnThisPanel.clear();
         graphsOnThisPanel = null;
      }

      selectedVariableHolder = null;

      this.removeAll();
   }

   @Override
   public void removeGraph(YoGraph graph)
   {
      this.graphsOnThisPanel.remove(graph);
      this.remove(graph);
      this.updateGraphs();
   }

   public void removeAllGraphs()
   {
      this.graphsOnThisPanel.clear();
      this.removeAll();
      this.updateGraphs();
   }

   public JPanel createGraphButtonPanel()
   {
      JPanel graphButtonPanel = new JPanel();

      JButton newGraphButton = new JButton("New Graph");
      newGraphButton.setName("New Graph");
      newGraphButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override
         public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addEmptyGraph();
         }
      });
      graphButtonPanel.add(newGraphButton);

      JButton removeEmptyGraphsButton = new JButton("Remove Empty");
      removeEmptyGraphsButton.setName("Remove Empty");
      removeEmptyGraphsButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override
         public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            removeEmptyGraphs();
         }
      });
      graphButtonPanel.add(removeEmptyGraphsButton);

      JButton addColumnButton = new JButton("Add Column");
      addColumnButton.setName("Add Column");
      addColumnButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override
         public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addColumn();
         }
      });
      graphButtonPanel.add(addColumnButton);

      JButton subColumnButton = new JButton("Sub Column");
      subColumnButton.setName("Sub Column");
      subColumnButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override
         public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            subColumn();
         }
      });
      graphButtonPanel.add(subColumnButton);
      
      JButton new3DGraphButton = new JButton("New 3D Graph");
      new3DGraphButton.setName("New 3D Graph");
      new3DGraphButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override
         public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addNew3dGraph();
         }
      });
      graphButtonPanel.add(new3DGraphButton);

      return graphButtonPanel;
   }

   public String getXMLRepresentationOfClass()
   {
      String returnString = "<GraphGroup>\n";
      returnString += "\t<Cols>" + numColumns + "</Cols>\n";

      for (int j = 0; j < graphsOnThisPanel.size(); j++)
      {
         YoGraph graph = graphsOnThisPanel.get(j);
         returnString += "\t<Graph>\n";
         returnString += "\t\t<Variables>";

         if (graph.getEntriesOnThisGraph().size() > 0)
         {
            returnString += graph.getEntriesOnThisGraph().get(0).getFullVariableNameWithNameSpace();

            for (int i = 1; i < graph.getEntriesOnThisGraph().size(); i++)
            {
               returnString += "," + graph.getEntriesOnThisGraph().get(i).getFullVariableNameWithNameSpace();
            }
         }

         returnString += "</Variables>";
         returnString += "\n" + graph.getGraphConfiguration().getXMLStyleRepresentationOfClass();
         returnString += "\n\t</Graph>\n";
      }

      returnString += "</GraphGroup>";

      return returnString;
   }

   @Override
   public ArrayList<Integer> getKeyPoints()
   {
      return dataBuffer.getKeyPoints();
   }
   
   private void updateGraphs()
   {
      //TODO: Why does this need to be updateUI instead of repaint()?
//      this.repaint();
      this.updateUI();
   }
}
