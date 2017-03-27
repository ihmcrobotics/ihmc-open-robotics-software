package us.ihmc.simulationConstructionSetTools.util.graphs;

import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JFrame;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;

public class GraphTester
{
   private final YoVariableRegistry registry = new YoVariableRegistry("registry");
   private final DataBuffer buffer = new DataBuffer(10);
   private final DoubleYoVariable xPlot = new DoubleYoVariable("xPlot", registry);
   private final DoubleYoVariable yPlot = new DoubleYoVariable("yPlot", registry);

   public GraphTester()
   {
      try
      {
         buffer.addVariable(xPlot);
         buffer.addVariable(yPlot);
      }
      catch (RepeatDataBufferEntryException e)
      {
         e.printStackTrace();
      }


      setUpGraph();
   }

   private void setupData(double divisor)
   {
      buffer.setIndex(0);

      for (double i = 1; i < 150; i++)
      {
         xPlot.set(i);
         yPlot.set(Math.sin(i / divisor));


         buffer.tickAndUpdate();
      }

   }

   private void setUpGraph()
   {
      try
      {
         setupData(1);
         JFreePlot plot1 = new JFreePlot(buffer.getEntry(xPlot), buffer.getEntry(yPlot));
         setupData(2);
         JFreePlot plot2 = new JFreePlot(buffer.getEntry(xPlot), buffer.getEntry(yPlot));
         setupData(4);
         JFreePlot plot3 = new JFreePlot(buffer.getEntry(xPlot), buffer.getEntry(yPlot));
         setupData(8);
         JFreePlot plot4 = new JFreePlot(buffer.getEntry(xPlot), buffer.getEntry(yPlot));
         plot1.setColor(Color.GREEN);
         plot1.setType(JFreePlot.PlotTypes.Dot);
         plot2.setColor(Color.BLUE);
         plot2.setType(JFreePlot.PlotTypes.Dash);
         plot3.setColor(Color.BLACK);
         plot3.setType(JFreePlot.PlotTypes.Solid);
         plot4.setColor(Color.RED);
         plot4.setType(JFreePlot.PlotTypes.Dot);
         JFreeGraph graph = new JFreeGraph("x vs y", "x", "y");
         graph.addPlot(plot1);
//         graph.addPlot(plot2);
//         graph.addPlot(plot3);
//         graph.addPlot(plot4);
         graph.setXAxisRange(30, 40);

         graph.setXAxisTickUnit(1);

         graph.setYAxisTickUnit(0.1);

         // graph.saveToJPG(new File("testImage.jpg"), 1024, 768);

         // graph.saveToSVG(new File("testImage.svg"));

         // graph.saveToPDF(new File("testImage.pdf"));

         setupData(1);
         JFreeGraph graph2 = JFreeGraph.createDataVsTimeGraph(buffer.getEntry(xPlot), buffer.getEntry(yPlot), Color.RED);

         setupData(2);
         JFreeGraph graph3 = JFreeGraph.createDataVsTimeGraph(buffer.getEntry(xPlot), buffer.getEntry(yPlot), Color.GREEN);

         setupData(3);
         JFreeGraph graph4 = JFreeGraph.createDataVsTimeGraph(buffer.getEntry(xPlot), buffer.getEntry(yPlot), Color.BLUE);

         setupData(4);
         JFreeGraph graph5 = JFreeGraph.createDataVsTimeGraph(buffer.getEntry(xPlot), buffer.getEntry(yPlot), Color.ORANGE);

         setupData(5);
         JFreeGraph graph6 = JFreeGraph.createDataVsTimeGraph(buffer.getEntry(xPlot), buffer.getEntry(yPlot));
         graph6.setTitle("New Title");


         JFrame testframe = new JFrame();

         ArrayList<JFreeGraph> graphs = new ArrayList<JFreeGraph>();

         graphs.add(graph);
         graphs.add(graph2);
         graphs.add(graph3);
         graphs.add(graph4);
         graphs.add(graph5);
         graphs.add(graph6);
         graphs.add(graph6.clone());
         graphs.add(graph6.clone());
         graphs.add(graph6.clone());


         graph.setXAxisTickUnit(100);

         JFreeGraphGroup group = new JFreeGraphGroup(graphs);
         


         testframe.setSize(1024, 768);
         testframe.getContentPane().add(group);
         testframe.setVisible(true);
         testframe.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);




         group.saveToJPEG("newImage");
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      new GraphTester();
   }


}
