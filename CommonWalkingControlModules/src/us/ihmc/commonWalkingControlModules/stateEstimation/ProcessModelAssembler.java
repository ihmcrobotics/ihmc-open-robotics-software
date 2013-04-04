package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.jgrapht.UndirectedGraph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.SimpleGraph;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public class ProcessModelAssembler
{
   private final UndirectedGraph<ProcessModelElement, ProcessModelGraphEdge> continuousTimeElementGraph = new SimpleGraph<ProcessModelElement, ProcessModelGraphEdge>(
         ProcessModelGraphEdge.class);
   private final ConnectivityInspector<ProcessModelElement, ProcessModelGraphEdge> connectivityInspector = new ConnectivityInspector<ProcessModelElement, ProcessModelGraphEdge>(
         continuousTimeElementGraph);
   private final List<ProcessModelElement> discreteTimeProcessModelElements = new ArrayList<ProcessModelElement>();

   public ProcessModel getProcessModel()
   {
      return new ProcessModel();
   }

   public void addProcessModelElement(ProcessModelElement processModelElement, ControlFlowOutputPort<?> statePort)
   {
      switch (processModelElement.getTimeDomain())
      {
      case CONTINUOUS:
      {
         registerContinuousTimeProcessModelElement(processModelElement);
         break;
      }
      case DISCRETE:
      {
         registerDiscreteTimeProcessModelElement(processModelElement);
         break;
      }
      default:
         throw new RuntimeException("Time domain not recognized");
      }
   }

   private void registerContinuousTimeProcessModelElement(ProcessModelElement processModelElement)
   {
      continuousTimeElementGraph.addVertex(processModelElement);

      Set<ControlFlowOutputPort<?>> statesInvolved = getStatesInvolved(processModelElement);
      Set<ControlFlowInputPort<?>> inputs = processModelElement.getInputs();

      for (ProcessModelElement otherProcessModelElement : continuousTimeElementGraph.vertexSet())
      {
         if (processModelElement != otherProcessModelElement)
         {
            Set<ControlFlowOutputPort<?>> otherStatesInvolved = getStatesInvolved(otherProcessModelElement);
            Set<ControlFlowOutputPort<?>> stateIntersection = new HashSet<ControlFlowOutputPort<?>>(statesInvolved);
            stateIntersection.retainAll(otherStatesInvolved);
            boolean stateCouplingExists = stateIntersection.size() > 0;

            Set<ControlFlowInputPort<?>> otherInputs = otherProcessModelElement.getInputs();
            Set<ControlFlowInputPort<?>> inputIntersection = new HashSet<ControlFlowInputPort<?>>(inputs);
            inputIntersection.retainAll(otherInputs);
            boolean inputCouplingExists = inputIntersection.size() > 0;

            if (stateCouplingExists || inputCouplingExists)
            {
               ProcessModelGraphEdge edge = new ProcessModelGraphEdge(stateIntersection, inputIntersection);
               continuousTimeElementGraph.addEdge(processModelElement, otherProcessModelElement, edge);
            }
         }
      }
   }

   private void registerDiscreteTimeProcessModelElement(ProcessModelElement processModelElement)
   {
      discreteTimeProcessModelElements.add(processModelElement);
   }

   private Set<ControlFlowOutputPort<?>> getStatesInvolved(ProcessModelElement processModelElement)
   {
      Set<ControlFlowOutputPort<?>> allStatesInvolved = new HashSet<ControlFlowOutputPort<?>>(processModelElement.getInputStates());
      allStatesInvolved.add(processModelElement.getOutputState());
      return allStatesInvolved;
   }

   public static class ProcessModelGroup implements ProcessModelElement
   {
      private final List<ProcessModelElement> processModelElements;
      private final TimeDomain timeDomain;
      private final boolean isTimeVariant;
      private final int size;

      public ProcessModelGroup(Collection<ProcessModelElement> processModelElements)
      {
         this.processModelElements = new ArrayList<ProcessModelElement>(processModelElements);
         this.timeDomain = determineTimeDomain(processModelElements);
         this.isTimeVariant = determineTimeVariant(processModelElements);
         this.size = determineSize(processModelElements);
      }

      private static TimeDomain determineTimeDomain(Collection<ProcessModelElement> processModelElements)
      {
         TimeDomain timeDomain = processModelElements.iterator().next().getTimeDomain();
         for (ProcessModelElement processModelElement : processModelElements)
         {
            if (processModelElement.getTimeDomain() != timeDomain)
               throw new RuntimeException("Cannot mix time domains in " + ProcessModelGroup.class.getSimpleName());
         }
         return timeDomain;
      }

      private static int determineSize(Collection<ProcessModelElement> processModelElements)
      {
         int ret = 0;
         for (ProcessModelElement processModelElement : processModelElements)
         {
            ret += processModelElement.getSize();
         }
         return ret;
      }

      private static boolean determineTimeVariant(Collection<ProcessModelElement> processModelElements)
      {
         boolean isTimeVariant = false;
         for (ProcessModelElement processModelElement : processModelElements)
         {
            if (processModelElement.isTimeVariant())
               isTimeVariant = true;
         }
         return isTimeVariant;
      }

      public List<ProcessModelElement> getProcessModelElements()
      {
         return processModelElements;
      }

      public boolean isTimeVariant()
      {
         return isTimeVariant;
      }

      public int getSize()
      {
         return size;
      }

      public void computeMatrixBlocks()
      {
         for (ProcessModelElement processModelElement : processModelElements)
         {
            processModelElement.computeMatrixBlocks();
         }
      }

      public DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort)
      {
         // TODO Auto-generated method stub
         return null;
      }

      public DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort)
      {
         // TODO Auto-generated method stub
         return null;
      }

      public DenseMatrix64F getProcessCovarianceMatrixBlock()
      {
         // TODO Auto-generated method stub
         return null;
      }

      public void propagateState(double dt)
      {
         for (ProcessModelElement processModelElement : processModelElements)
         {
            processModelElement.propagateState(dt);
         }
      }

      public void correctState(DenseMatrix64F correction)
      {
         // TODO Auto-generated method stub
         
      }

      public TimeDomain getTimeDomain()
      {
         return timeDomain;
      }

      public Set<ControlFlowOutputPort<?>> getInputStates()
      {
         // TODO Auto-generated method stub
         return null;
      }

      public ControlFlowOutputPort<?> getOutputState()
      {
         // TODO Auto-generated method stub
         return null;
      }

      public Set<ControlFlowInputPort<?>> getInputs()
      {
         // TODO Auto-generated method stub
         return null;
      }
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
