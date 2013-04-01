package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import java.util.HashMap;

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

   public AbstractProcessModelElement(TimeDomain timeDomain, int covarianceMatrixSize, String name, YoVariableRegistry registry)
   {
      stateMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
      inputMatrixBlocks = new HashMap<ControlFlowInputPort<?>, DenseMatrix64F>();
      processNoiseCovarianceBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      scaledProcessNoiseCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      covarianceMatrixScaling.set(1.0);
      this.timeDomain = timeDomain;
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
}
