package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import java.util.HashMap;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.ProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.TimeDomain;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class AbstractProcessModelElement implements ProcessModelElement
{
   protected final HashMap<ControlFlowOutputPort<?>, DenseMatrix64F> stateMatrixBlocks;
   protected final HashMap<ControlFlowInputPort<?>, DenseMatrix64F> inputMatrixBlocks;
   private final DenseMatrix64F processNoiseCovarianceBlock;
   private final DenseMatrix64F scaledProcessNoiseCovarianceMatrixBlock;
   private final DoubleYoVariable covarianceMatrixScaling;
   private final TimeDomain timeDomain;
   private final boolean isTimeVariant;
   private final ControlFlowOutputPort<?> outputState;

   public AbstractProcessModelElement(ControlFlowOutputPort<?> outputState, TimeDomain timeDomain, boolean isTimeVariant, int size, String name,
         YoVariableRegistry registry)
   {
      this.outputState = outputState;
      this.stateMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
      this.inputMatrixBlocks = new HashMap<ControlFlowInputPort<?>, DenseMatrix64F>();
      this.processNoiseCovarianceBlock = new DenseMatrix64F(size, size);
      this.scaledProcessNoiseCovarianceMatrixBlock = new DenseMatrix64F(size, size);
      this.covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      this.covarianceMatrixScaling.set(1.0);
      this.timeDomain = timeDomain;
      this.isTimeVariant = isTimeVariant;
   }

   public DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return stateMatrixBlocks.get(statePort);
   }

   public DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort)
   {
      return inputMatrixBlocks.get(inputPort);
   }

   public DenseMatrix64F getProcessCovarianceMatrixBlock()
   {
      scaledProcessNoiseCovarianceMatrixBlock.set(processNoiseCovarianceBlock);
      CommonOps.scale(covarianceMatrixScaling.getDoubleValue(), scaledProcessNoiseCovarianceMatrixBlock);

      return scaledProcessNoiseCovarianceMatrixBlock;
   }

   public void setProcessNoiseCovarianceBlock(DenseMatrix64F processNoiseCovarianceBlock)
   {
      this.processNoiseCovarianceBlock.set(processNoiseCovarianceBlock);
   }

   public void setProcessNoiseStandardDeviation(double standardDeviation)
   {
      double variance = MathTools.square(standardDeviation);
      CommonOps.setIdentity(processNoiseCovarianceBlock);
      CommonOps.scale(variance, processNoiseCovarianceBlock);
   }

   public TimeDomain getTimeDomain()
   {
      return timeDomain;
   }

   public Set<ControlFlowOutputPort<?>> getInputStates()
   {
      return stateMatrixBlocks.keySet();
   }

   public Set<ControlFlowInputPort<?>> getInputs()
   {
      return inputMatrixBlocks.keySet();
   }

   public boolean isTimeVariant()
   {
      return isTimeVariant;
   }

   public ControlFlowOutputPort<?> getOutputState()
   {
      return outputState;
   }
}
