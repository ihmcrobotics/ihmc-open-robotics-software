package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public interface ProcessModelElement
{
   public abstract String getName();
   
   public abstract void computeMatrixBlocks();

   public abstract DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort);

   public abstract DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort);

   public abstract DenseMatrix64F getProcessCovarianceMatrixBlock();

   public abstract void propagateState(double dt);

   public abstract void correctState(DenseMatrix64F correction);

   public abstract TimeDomain getTimeDomain();

   public abstract Set<ControlFlowOutputPort<?>> getInputStates();
   
   public abstract ControlFlowOutputPort<?> getOutputState();

   public abstract Set<ControlFlowInputPort<?>> getInputs();

   public abstract boolean isTimeVariant();
}
