package us.ihmc.controlFlow;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;

public class ControlFlowGraph implements ControlFlowElement
{
   private final LinkedHashMap<ControlFlowElement, ControlFlowNode> elementsToNodes = new LinkedHashMap<ControlFlowElement, ControlFlowNode>();
   private ArrayList<ControlFlowNode> nodesInOrderOfGraphExecution;

   private ArrayList<ControlFlowInputPort<?>> inputPorts = new ArrayList<ControlFlowInputPort<?>>();
   private ArrayList<ControlFlowOutputPort<?>> outputPorts = new ArrayList<ControlFlowOutputPort<?>>();

   private final LinkedHashMap<ControlFlowInputPort<?>, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>>> inputPortsConnections =
      new LinkedHashMap<ControlFlowInputPort<?>, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>>>();
   private final LinkedHashMap<ControlFlowOutputPort<?>, ImmutablePair<ControlFlowNode, ControlFlowOutputToOutputPortConnector<?>>> outputPortsConnections =
      new LinkedHashMap<ControlFlowOutputPort<?>, ImmutablePair<ControlFlowNode, ControlFlowOutputToOutputPortConnector<?>>>();

   public ControlFlowGraph()
   {
   }

   public <DataType> void connectInputPort(ControlFlowInputPort<DataType> graphInputPort, ControlFlowInputPort<DataType> elementInputPort)
   {
      ControlFlowElement controlFlowElement = elementInputPort.getControlFlowElement();
      ControlFlowNode controlFlowNode = getOrCreateNode(controlFlowElement);

      if (!inputPorts.contains(graphInputPort))
      {
         inputPorts.add(graphInputPort);
         ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>> inputPortConnections = new ArrayList<ImmutablePair<ControlFlowNode,
                                                                                                             ControlFlowInputToInputPortConnector<?>>>();
         inputPortsConnections.put(graphInputPort, inputPortConnections);
      }

      ControlFlowInputToInputPortConnector<DataType> inputToInputConnector = new ControlFlowInputToInputPortConnector<DataType>(graphInputPort,
                                                                                elementInputPort);
      ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>> newConnection = new ImmutablePair<ControlFlowNode,
                                                                                        ControlFlowInputToInputPortConnector<?>>(controlFlowNode,
                                                                                           inputToInputConnector);

      ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>> inputPortConnections = inputPortsConnections.get(graphInputPort);
      inputPortConnections.add(newConnection);

   }

   public <DataType> void connectOutputPort(ControlFlowOutputPort<DataType> graphOutputPort, ControlFlowOutputPort<DataType> elementsOutputPort)
   {
      ControlFlowElement controlFlowElement = elementsOutputPort.getControlFlowElement();
      ControlFlowNode controlFlowNode = getOrCreateNode(controlFlowElement);

      if (outputPorts.contains(graphOutputPort))
      {
         throw new RuntimeException("Already connected output port " + graphOutputPort + " for this graph!");
      }

      outputPorts.add(graphOutputPort);

      ControlFlowOutputToOutputPortConnector<DataType> outputToOutputConnector = new ControlFlowOutputToOutputPortConnector<DataType>(elementsOutputPort,
                                                                                    graphOutputPort);
      ImmutablePair<ControlFlowNode, ControlFlowOutputToOutputPortConnector<?>> outputPortConnections = new ImmutablePair<ControlFlowNode,
                                                                                                  ControlFlowOutputToOutputPortConnector<?>>(controlFlowNode,
                                                                                                     outputToOutputConnector);

      outputPortsConnections.put(graphOutputPort, outputPortConnections);
   }


   public void initializeAfterConnections()
   {
      if (nodesInOrderOfGraphExecution != null)
      {
         throw new RuntimeException("Already called initializeAfterConnections!");
      }

      // Make sure all the input ports are connected.

      for (ControlFlowInputPort<?> inputPort : inputPorts)
      {
         ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>> inputPortConnections = inputPortsConnections.get(inputPort);

         if ((inputPortConnections == null) || (inputPortConnections.isEmpty()))
         {
            throw new RuntimeException("Input Port " + inputPort + " has not been connected to anything");
         }
      }

      // Make sure all the output ports are connected.

      for (ControlFlowOutputPort<?> outputPort : outputPorts)
      {
         ImmutablePair<ControlFlowNode, ControlFlowOutputToOutputPortConnector<?>> outputPortConnection = outputPortsConnections.get(outputPort);
         if (outputPortConnection == null)
         {
            throw new RuntimeException("Output Port " + outputPort + " has not been assigned!");
         }
      }

      nodesInOrderOfGraphExecution = new ArrayList<ControlFlowNode>();
      ConcurrentLinkedQueue<ControlFlowNode> nodesToExpand = new ConcurrentLinkedQueue<ControlFlowNode>();

      // Mark all the input ports:
      Collection<ControlFlowNode> allNodes = elementsToNodes.values();
      addAllNodesWithoutInputPorts(allNodes, nodesToExpand);
      markNodesFromInputsAndAddIfAllInputPortsAreMarked(inputPortsConnections, nodesToExpand);

      while (!nodesToExpand.isEmpty())
      {
         ControlFlowNode nodeToExpand = nodesToExpand.poll();

         if (nodesInOrderOfGraphExecution.contains(nodeToExpand))
         {
            throw new RuntimeException("Control Flow Graph contains a loop!!");
         }

         nodesInOrderOfGraphExecution.add(nodeToExpand);

         Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts =
            nodeToExpand.getConnectionsFromOutputPorts();

         if (connectionsFromOutputPorts != null)
         {
            markNodesFromOutputsAndAddIfAllInputPortsAreMarked(connectionsFromOutputPorts, nodesToExpand);
         }
      }

      // Make sure all nodes got marked:
      
      
      
      if (allNodes.size() != nodesInOrderOfGraphExecution.size())
      {
         throw new RuntimeException("Not all nodes got connected! allNodes.size() = " + allNodes.size() + ", nodesInOrderOfGraphExecution.size() = " + nodesInOrderOfGraphExecution.size());
      }

      for (ControlFlowNode node : allNodes)
      {
         if (!node.areAllInputPortsAttached())
         {
            throw new RuntimeException("An input port isn't attached on control flow node " + node);
         }
      }
      

      setInputPortData();

      // Propagate stuff through:
      for(int i = 0; i <  nodesInOrderOfGraphExecution.size(); i++)
      {
         ControlFlowNode controlFlowNode  = nodesInOrderOfGraphExecution.get(i);
         controlFlowNode.initialize();

         Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts =
            controlFlowNode.getConnectionsFromOutputPorts();

         for (ControlFlowOutputPort<?> outputPort : connectionsFromOutputPorts.keySet())
         {
            ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>> connectionsFromOutputPort = connectionsFromOutputPorts.get(outputPort);

            if (connectionsFromOutputPort != null)
            {
               for(int j = 0; j <  connectionsFromOutputPort.size(); j++)
               {
//                ControlFlowNode connectedNode = connectionFromOutputPort.first();
                  ControlFlowOutputToInputPortConnector<?> connector = connectionsFromOutputPort.get(j).getRight();
                  connector.sendDataAlongConnector();
               }
            }
         }
      }

      setOutputPortData();
   }


   private void addAllNodesWithoutInputPorts(Collection<ControlFlowNode> nodes, ConcurrentLinkedQueue<ControlFlowNode> nodesToExpand)
   {
      for (ControlFlowNode node : nodes)
      {
         if (node.containsNoInputPorts())
         {
            nodesToExpand.add(node);
         }
      }
      
   }

   private static void markNodesFromOutputsAndAddIfAllInputPortsAreMarked(Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts, ConcurrentLinkedQueue<ControlFlowNode> nodesToExpand)
   {

      for (ControlFlowOutputPort<?> outputPort : connectionsFromOutputPorts.keySet())
      {
         ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>> portConnections = connectionsFromOutputPorts.get(outputPort);
         if (portConnections != null)
         {
            for (ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>> portConnection : portConnections)
            {
               ControlFlowNode node = portConnection.getLeft();
               node.setInputPortIsAttached(portConnection.getRight().getInputPort());

               if (node.areAllInputPortsAttached())
               {
                  if (!nodesToExpand.contains(node))
                  {
                     nodesToExpand.add(node);
                  }
               }
            }
         }
      }
   }

   private static void markNodesFromInputsAndAddIfAllInputPortsAreMarked(HashMap<ControlFlowInputPort<?>,
           ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>>> connectionsToExpand, ConcurrentLinkedQueue<ControlFlowNode> nodesToExpand)
   {
      Set<ControlFlowInputPort<?>> inputPorts = connectionsToExpand.keySet();

      for (ControlFlowInputPort<?> inputPort : inputPorts)
      {
         ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>> portConnections = connectionsToExpand.get(inputPort);
         if (portConnections != null)
         {
            for (ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>> portConnection : portConnections)
            {
               ControlFlowNode node = portConnection.getLeft();
               node.setInputPortIsAttached(portConnection.getRight().getToInputPort());

               if (node.areAllInputPortsAttached())
               {
                  if (!nodesToExpand.contains(node))
                  {
                     nodesToExpand.add(node);
                  }
               }
            }
         }
      }
   }

   public <DataType> void connectElements(ControlFlowOutputPort<DataType> elementOneOutputPort, ControlFlowInputPort<DataType> elementTwoInputPort)
   {
      ControlFlowElement elementOne = elementOneOutputPort.getControlFlowElement();
      ControlFlowElement elementTwo = elementTwoInputPort.getControlFlowElement();

      ControlFlowNode nodeOne = getOrCreateNode(elementOne);
      ControlFlowNode nodeTwo = getOrCreateNode(elementTwo);

      nodeOne.connectOutputPortTo(elementOneOutputPort, nodeTwo, elementTwoInputPort);
   }

   private ControlFlowNode getOrCreateNode(ControlFlowElement controlFlowElement)
   {
      ControlFlowNode controlFlowNode = elementsToNodes.get(controlFlowElement);
      if (controlFlowNode == null)
      {
         controlFlowNode = new ControlFlowNode(controlFlowElement);
         elementsToNodes.put(controlFlowElement, controlFlowNode);
      }

      return controlFlowNode;
   }

   public void visualize()
   {
      ControlFlowVisualizer visualizer = new ControlFlowVisualizer(this);
      visualizer.visualize();
   }

   protected Collection<ControlFlowNode> getNodes()
   {
      return elementsToNodes.values();
   }

   public void startComputation()
   {
      if (nodesInOrderOfGraphExecution == null)
      {
         throw new RuntimeException("Forgot to call initializeAfterConnections()");
      }

      setInputPortData();

      // Propagate stuff through:
      for(int i = 0; i <  nodesInOrderOfGraphExecution.size(); i++)
      {
         ControlFlowNode controlFlowNode  = nodesInOrderOfGraphExecution.get(i);
         controlFlowNode.startComputation();
         controlFlowNode.waitUntilComputationIsDone();

         Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts =
               controlFlowNode.getConnectionsFromOutputPorts();

         for (ControlFlowOutputPort<?> outputPort : connectionsFromOutputPorts.keySet())
         {
            ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>> connectionsFromOutputPort = connectionsFromOutputPorts.get(outputPort);

            if (connectionsFromOutputPort != null)
            {
               for(int j = 0; j <  connectionsFromOutputPort.size(); j++)
               {
                  //                ControlFlowNode connectedNode = connectionFromOutputPort.first();
                  ControlFlowOutputToInputPortConnector<?> connector = connectionsFromOutputPort.get(j).getRight();
                  connector.sendDataAlongConnector();
               }
            }
         }
      }

      setOutputPortData();
   }


   private void setInputPortData()
   {
      for(int i = 0; i <  inputPorts.size(); i++)
      {
         ArrayList<ImmutablePair<ControlFlowNode, ControlFlowInputToInputPortConnector<?>>> inputPortConnections = inputPortsConnections.get(inputPorts.get(i));

         for(int j = 0; j <  inputPortConnections.size(); j++)
         {
//          ControlFlowNode controlFlowNode = inputPortConnection.first();
//          ControlFlowElement controlFlowElement = controlFlowNode.getControlFlowElement();
            ControlFlowInputToInputPortConnector<?> inputPortConnector = inputPortConnections.get(j).getRight();

            inputPortConnector.sendDataAlongConnector();
         }
      }
   }

   private void setOutputPortData()
   {
      for(int i = 0; i <  outputPorts.size(); i++)
      {
         ImmutablePair<ControlFlowNode, ControlFlowOutputToOutputPortConnector<?>> outputPortConnection = outputPortsConnections.get(outputPorts.get(i));

//       ControlFlowNode controlFlowNode = outputPortConnection.first();
//       ControlFlowElement controlFlowElement = controlFlowNode.getControlFlowElement();
         ControlFlowOutputToOutputPortConnector<?> outputPortConnector = outputPortConnection.getRight();
         outputPortConnector.sendDataAlongConnector();
      }
   }

   public void waitUntilComputationIsDone()
   {
   }

   public List<ControlFlowInputPort<?>> getInputPorts()
   {
      return inputPorts;
   }

   public List<ControlFlowOutputPort<?>> getOutputPorts()
   {
      return outputPorts;
   }

   public void initialize()
   {
//    empty
   }
}
