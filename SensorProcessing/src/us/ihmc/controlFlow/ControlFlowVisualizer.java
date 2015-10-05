package us.ihmc.controlFlow;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.jgraph.JGraph;
import org.jgraph.graph.DefaultEdge;
import org.jgrapht.ListenableGraph;
import org.jgrapht.ext.JGraphModelAdapter;
import org.jgrapht.graph.ListenableDirectedGraph;


public class ControlFlowVisualizer extends JPanel
{
   private static final long serialVersionUID = -8937004613741097094L;
 
   private JGraphModelAdapter<ControlFlowElementVisualizer, DefaultEdge> jGraphModelAdapter;

   private final ControlFlowGraph controlFlowGraph;
  
   private final LinkedHashMap<ControlFlowElement, ControlFlowElementVisualizer> elementsToVisualizers = new LinkedHashMap<ControlFlowElement, ControlFlowElementVisualizer>();

   public ControlFlowVisualizer(ControlFlowGraph controlFlowGraph)
   {
      this.controlFlowGraph = controlFlowGraph;
   }


   public void visualize()
   {   
      ListenableGraph<ControlFlowElementVisualizer, DefaultEdge> graph = new ListenableDirectedGraph<ControlFlowElementVisualizer, DefaultEdge>(DefaultEdge.class );

      // create a visualization using JGraph, via an adapter
      jGraphModelAdapter = new JGraphModelAdapter<ControlFlowElementVisualizer, DefaultEdge>(graph);

      JGraph jGraph = new JGraph(jGraphModelAdapter);

//      DirectedGraph<String, DefaultEdge> foo = new DefaultDirectedGraph<String, DefaultEdge>(DefaultEdge.class);
      
      Collection<ControlFlowNode> controlFlowNodes = controlFlowGraph.getNodes();
    
      // First add the vertices:
    for (ControlFlowNode node : controlFlowNodes)
    {
       ControlFlowElement controlFlowElement = node.getControlFlowElement();
       ControlFlowElementVisualizer controlFlowElementVisualizer = new ControlFlowElementVisualizer(controlFlowElement);
       elementsToVisualizers.put(controlFlowElement, controlFlowElementVisualizer);
       graph.addVertex(controlFlowElementVisualizer);
    }
    
    // Then add the edges:
    for (ControlFlowNode node : controlFlowNodes)
    {
       ControlFlowElement controlFlowElement = node.getControlFlowElement();
       Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts = node.getConnectionsFromOutputPorts();

       for (ControlFlowOutputPort<?> outputPort : connectionsFromOutputPorts.keySet())
       {
          ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>> connectionsFromOutputPort = connectionsFromOutputPorts.get(outputPort);

          for (ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>> connectionFromOutputPort : connectionsFromOutputPort)
          {
             ControlFlowNode connectedNode = connectionFromOutputPort.getLeft();
             
             ControlFlowElementVisualizer controlFlowElementVisualizer = elementsToVisualizers.get(controlFlowElement);
             ControlFlowElementVisualizer connectedNodeControlFlowElementVisualizer = elementsToVisualizers.get(connectedNode.getControlFlowElement());

             graph.addEdge(controlFlowElementVisualizer, connectedNodeControlFlowElementVisualizer);
          }
       }  
    }

    JFrame jFrame = new JFrame("ControlFlowVisualizer");

    jFrame.pack();
    jFrame.setSize(800, 600);
    jFrame.setVisible(true);

    jFrame.getContentPane().add(jGraph);

   }
   
//   public void paintComponent(Graphics graphics)
//   {
//      super.paintComponent(graphics);
//      
//      Collection<ControlFlowNode<?,?>> controlFlowNodes = controlFlowGraph.getNodes();
//      
//      for (ControlFlowNode<?, ?> node : controlFlowNodes)
//      {
//         ControlFlowElement controlFlowElement = node.getControlFlowElement();
//         ArrayList<ControlFlowNode<?,?>> connectionsFromOutport = node.getConnectionsFromOutport();
//      }
//   }
   
//   private void recursivePaintComponent()
//   {
//      
//   }
   
   
//   private void positionVertexAt(Object vertex, int x, int y) 
//   {
//      DefaultGraphCell cell = jGraphModelAdapter.getVertexCell(vertex);
//      Map attr = cell.getAttributes();
//      Rectangle2D b = GraphConstants.getBounds(attr);
//
//      GraphConstants.setBounds(attr, new Rectangle2D(x, y, b.getWidth(), b.getHeight()));
//
//      Map cellAttr = new LinkedHashMap();
//      cellAttr.put(cell, attr);
//      jGraphModelAdapter.edit( cellAttr, null, null, null);
//  }
   
   
   private class ControlFlowElementVisualizer
   {
      private final ControlFlowElement controlFlowElement;
      
      public ControlFlowElementVisualizer(ControlFlowElement controlFlowElement)
      {
         this.controlFlowElement = controlFlowElement;
      }
      
      public String toString()
      {
         Class<? extends ControlFlowElement> class1 = controlFlowElement.getClass();   
         String simpleName = class1.getSimpleName();
         
         List<ControlFlowInputPort<?>> inputPorts = controlFlowElement.getInputPorts();
         List<ControlFlowOutputPort<?>> outputPorts = controlFlowElement.getOutputPorts();
         
         return simpleName + "\n" + inputPorts.size();
      }
   }
}
