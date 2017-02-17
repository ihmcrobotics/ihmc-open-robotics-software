package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassVelocityDiscreteProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort;

   // temp stuff
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);
   private final FrameVector centerOfMassVelocityDelta = new FrameVector(ReferenceFrame.getWorldFrame());

   public CenterOfMassVelocityDiscreteProcessModelElement(double deltaT, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(centerOfMassVelocityPort, TimeDomain.DISCRETE, false, SIZE, name, registry);

      stateMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      stateMatrixBlocks.put(centerOfMassAccelerationPort, new DenseMatrix64F(SIZE, SIZE));

      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;

      computeCenterOfMassVelocityStateMatrixBlock();
      computeCenterOfMassAccelerationStateMatrixBlock(deltaT);
   }

   private void computeCenterOfMassVelocityStateMatrixBlock()
   {
      DenseMatrix64F stateMatrixBlock = stateMatrixBlocks.get(centerOfMassVelocityPort);
      CommonOps.setIdentity(stateMatrixBlock);
   }

   private void computeCenterOfMassAccelerationStateMatrixBlock(double deltaT)
   {
      DenseMatrix64F stateMatrixBlock = stateMatrixBlocks.get(centerOfMassAccelerationPort);
      CommonOps.setIdentity(stateMatrixBlock);
      CommonOps.scale(deltaT, stateMatrixBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      centerOfMassVelocityDelta.set(centerOfMassAccelerationPort.getData());
      centerOfMassVelocityDelta.scale(dt);

      updateCenterOfMassVelocity(centerOfMassVelocityDelta);
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(centerOfMassVelocityDelta.getVector(), correction, 0);
      updateCenterOfMassVelocity(centerOfMassVelocityDelta);
   }

   private void updateCenterOfMassVelocity(FrameVector centerOfMassVelocityDelta)
   {
      centerOfMassVelocity.set(centerOfMassVelocityPort.getData());
      centerOfMassVelocity.add(centerOfMassVelocityDelta);
      centerOfMassVelocityPort.setData(centerOfMassVelocity);
   }
}
