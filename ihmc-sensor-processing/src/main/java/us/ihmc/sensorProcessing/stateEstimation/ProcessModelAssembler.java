package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import org.jgrapht.UndirectedGraph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.SimpleGraph;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;

public class ProcessModelAssembler
{
   private final UndirectedGraph<ProcessModelElement, ProcessModelGraphEdge> processModelElementGraph = new SimpleGraph<ProcessModelElement, ProcessModelGraphEdge>(
         ProcessModelGraphEdge.class);
   private final ConnectivityInspector<ProcessModelElement, ProcessModelGraphEdge> connectivityInspector = new ConnectivityInspector<ProcessModelElement, ProcessModelGraphEdge>(
         processModelElementGraph);
   private final double controlDT;

   public ProcessModelAssembler(double controlDT)
   {
      this.controlDT = controlDT;
   }

   public ProcessModel getProcessModel()
   {
      List<Set<ProcessModelElement>> connectedSets = connectivityInspector.connectedSets();
      List<ProcessModelElementGroup> processModelElementGroups = new ArrayList<ProcessModelElementGroup>();
      for (Set<ProcessModelElement> connectedSet : connectedSets)
      {
         ProcessModelElementGroup group = new ProcessModelElementGroup(connectedSet, controlDT);
         processModelElementGroups.add(group);
      }
      return new ProcessModel(processModelElementGroups);
   }

   public void addProcessModelElement(ProcessModelElement processModelElement, ControlFlowOutputPort<?> statePort)
   {
      processModelElementGraph.addVertex(processModelElement);

      Set<ControlFlowOutputPort<?>> statesInvolved = getStatesInvolved(processModelElement);
      Set<ControlFlowInputPort<?>> inputs = processModelElement.getInputs();

      for (ProcessModelElement otherProcessModelElement : processModelElementGraph.vertexSet())
      {
         if (processModelElement != otherProcessModelElement)
         {
            Set<ControlFlowOutputPort<?>> otherStatesInvolved = getStatesInvolved(otherProcessModelElement);
            Set<ControlFlowOutputPort<?>> stateIntersection = new LinkedHashSet<ControlFlowOutputPort<?>>(statesInvolved);
            stateIntersection.retainAll(otherStatesInvolved);
            boolean stateCouplingExists = stateIntersection.size() > 0;

            Set<ControlFlowInputPort<?>> otherInputs = otherProcessModelElement.getInputs();
            Set<ControlFlowInputPort<?>> inputIntersection = new LinkedHashSet<ControlFlowInputPort<?>>(inputs);
            inputIntersection.retainAll(otherInputs);
            boolean inputCouplingExists = inputIntersection.size() > 0;

            if (stateCouplingExists || inputCouplingExists)
            {
               ProcessModelGraphEdge edge = new ProcessModelGraphEdge(stateIntersection, inputIntersection);
               processModelElementGraph.addEdge(processModelElement, otherProcessModelElement, edge);
            }
         }
      }
   }

   private Set<ControlFlowOutputPort<?>> getStatesInvolved(ProcessModelElement processModelElement)
   {
      Set<ControlFlowOutputPort<?>> allStatesInvolved = new LinkedHashSet<ControlFlowOutputPort<?>>(processModelElement.getInputStates());
      allStatesInvolved.add(processModelElement.getOutputState());
      return allStatesInvolved;
   }

   public static class ProcessModelGraphEdge
   {
      private final Set<ControlFlowOutputPort<?>> commonStates;
      private final Set<ControlFlowInputPort<?>> commonInputs;

      public ProcessModelGraphEdge(Set<ControlFlowOutputPort<?>> commonStates, Set<ControlFlowInputPort<?>> commonInputs)
      {
         this.commonInputs = commonInputs;
         this.commonStates = commonStates;
      }

      public Set<ControlFlowOutputPort<?>> getCommonStates()
      {
         return commonStates;
      }

      public Set<ControlFlowInputPort<?>> getCommonInputs()
      {
         return commonInputs;
      }
   }
}
