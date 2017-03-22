package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import java.util.LinkedHashMap;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;


public abstract class AbstractProcessModelElement implements ProcessModelElement
{
   protected final LinkedHashMap<ControlFlowOutputPort<?>, DenseMatrix64F> stateMatrixBlocks;
   protected final LinkedHashMap<ControlFlowInputPort<?>, DenseMatrix64F> inputMatrixBlocks;
   private final DenseMatrix64F processNoiseCovarianceBlock;
   private final DenseMatrix64F scaledProcessNoiseCovarianceMatrixBlock;
   private final DoubleYoVariable covarianceMatrixScaling;
   private final TimeDomain timeDomain;
   private final boolean isTimeVariant;
   private final ControlFlowOutputPort<?> outputState;

   private final String name;
   
   public AbstractProcessModelElement(ControlFlowOutputPort<?> outputState, TimeDomain timeDomain, boolean isTimeVariant, int size, String name,
         YoVariableRegistry registry)
   {
      this.outputState = outputState;
      this.stateMatrixBlocks = new LinkedHashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
      this.inputMatrixBlocks = new LinkedHashMap<ControlFlowInputPort<?>, DenseMatrix64F>();
      this.processNoiseCovarianceBlock = new DenseMatrix64F(size, size);
      this.scaledProcessNoiseCovarianceMatrixBlock = new DenseMatrix64F(size, size);
      this.covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      this.covarianceMatrixScaling.set(1.0);
      this.timeDomain = timeDomain;
      this.isTimeVariant = isTimeVariant;
      this.name = name;
   }

   public String getName()
   {
      return name;
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
