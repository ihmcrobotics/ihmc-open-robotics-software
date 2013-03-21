package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.stateEstimation.ProcessModelElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;

public class BiasProcessModelElement implements ProcessModelElement
{
   private final ControlFlowOutputPort<FrameVector> biasPort;
   private final DenseMatrix64F covarianceMatrix;
   private final Vector3d bias = new Vector3d();
   private final Vector3d biasDelta = new Vector3d();

   public BiasProcessModelElement(ControlFlowOutputPort<FrameVector> statePort, int size)
   {
      this.biasPort = statePort;
      covarianceMatrix = new DenseMatrix64F(size);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return null;
   }

   public DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort)
   {
      return null;
   }

   public DenseMatrix64F getProcessCovarianceMatrixBlock()
   {
      return covarianceMatrix;
   }

   public void propagateState(double dt)
   {
      // empty
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(biasDelta, correction, 0);
      updateBias(biasDelta);
   }

   private void updateBias(Vector3d biasDelta)
   {
      biasPort.getData().getVector(bias);
      bias.add(biasDelta);
      biasPort.getData().set(bias);
   }
}
