package us.ihmc.controlFlow;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

@Deprecated
class ControlFlowNode implements ControlFlowElement
{
   private final ControlFlowElement controlFlowElement;

   private final Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> connectionsFromOutputPorts;
   private final LinkedHashMap<ControlFlowInputPort<?>, Boolean> inputPortIsAttached;
   
   public ControlFlowNode(ControlFlowElement controlFlowElement)
   {
      this.controlFlowElement = controlFlowElement;
                 
      inputPortIsAttached = new LinkedHashMap<ControlFlowInputPort<?>, Boolean>();
      connectionsFromOutputPorts = new HashMap<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>>();
   }
   
   public ControlFlowElement getControlFlowElement()
   {
      return controlFlowElement;
   }

   public void initialize()
   {
      controlFlowElement.initialize();
   }

   public void setInputPortIsAttached(ControlFlowInputPort<?> inputPort)
   {
      inputPortIsAttached.put(inputPort, true);
   }
   
   public boolean areAllInputPortsAttached()
   {
      Collection<Boolean> isInputPortAttachedCollection = inputPortIsAttached.values();
      
      for (Boolean isInputPortAttached : isInputPortAttachedCollection)
      {
         if (!isInputPortAttached) return false;
      }
      
      for (ControlFlowInputPort<?> inputPort : controlFlowElement.getInputPorts())
      {
         if (inputPortIsAttached.get(inputPort) == null) return false;
      }
      
      return true;
   }
   
   public <DataType> void connectOutputPortTo(ControlFlowOutputPort outputPort, ControlFlowNode connectionFromOutport, ControlFlowInputPort<DataType> inputPort)
   {
      ControlFlowOutputToInputPortConnector<DataType> connector = new ControlFlowOutputToInputPortConnector<DataType>(outputPort, inputPort);
      ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>> newConnection = new ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>(connectionFromOutport, connector);

      ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>> connectionsFromOutputPort = connectionsFromOutputPorts.get(outputPort);
      if (connectionsFromOutputPort == null)
      {
         if (controlFlowElement.getOutputPorts().contains(outputPort))
         {
            connectionsFromOutputPort = new ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>();
            connectionsFromOutputPorts.put(outputPort, connectionsFromOutputPort);
         }
         else
         {         
            throw new RuntimeException("Have you registered all ports? It seems as though port " + outputPort + " isn't one of the output ports of controlFlowElement " + controlFlowElement);
         }
      }

      connectionsFromOutputPort.add(newConnection);
   }

   public Map<ControlFlowOutputPort, ArrayList<ImmutablePair<ControlFlowNode, ControlFlowOutputToInputPortConnector<?>>>> getConnectionsFromOutputPorts()
   {
      return connectionsFromOutputPorts;
   }

   public void startComputation()
   {
      controlFlowElement.startComputation();
   }

   public void waitUntilComputationIsDone()
   {
      controlFlowElement.waitUntilComputationIsDone();
   }
   
   public boolean containsNoInputPorts()
   {
      return controlFlowElement.getInputPorts().isEmpty();
   }
   
   public List<ControlFlowInputPort<?>> getInputPorts()
   {
      return controlFlowElement.getInputPorts();
   }

   public List<ControlFlowOutputPort<?>> getOutputPorts()
   {
      return controlFlowElement.getOutputPorts();
   }
   
   public String toString()
   {
      return "ControlFlowNode: ControlFlowElement = " + controlFlowElement;
   }

}
