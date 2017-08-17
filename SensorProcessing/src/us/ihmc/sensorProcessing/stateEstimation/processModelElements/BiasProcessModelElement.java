package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class BiasProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> biasPort;
   private final FrameVector3D bias;
   private final FrameVector3D biasDelta;

   public BiasProcessModelElement(ControlFlowOutputPort<FrameVector3D> statePort, ReferenceFrame frame, String name, YoVariableRegistry registry)
   {
      super(statePort, TimeDomain.CONTINUOUS, false, SIZE, name, registry);
      this.biasPort = statePort;
      this.bias = new FrameVector3D(frame);
      this.biasDelta = new FrameVector3D(frame);
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
      MatrixTools.extractTuple3dFromEJMLVector(biasDelta.getVector(), correction, 0);
      updateBias(biasDelta);
   }

   private void updateBias(FrameVector3D biasDelta)
   {
      bias.set(biasPort.getData());
      bias.add(biasDelta);
      biasPort.setData(bias);
   }
}
