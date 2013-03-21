package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;

public class BiasProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> biasPort;
   private final Vector3d bias = new Vector3d();
   private final Vector3d biasDelta = new Vector3d();

   public BiasProcessModelElement(ControlFlowOutputPort<FrameVector> statePort, String name, YoVariableRegistry registry)
   {
      super(SIZE, 0, 0, name, registry);
      this.biasPort = statePort;
   }

   public void computeMatrixBlocks()
   {
      // empty
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
