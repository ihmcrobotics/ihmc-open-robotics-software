package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassVelocityDiscreteProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort;

   // temp stuff
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D centerOfMassVelocityDelta = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public CenterOfMassVelocityDiscreteProcessModelElement(double deltaT, ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort, String name, YoVariableRegistry registry)
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

   private void updateCenterOfMassVelocity(FrameVector3D centerOfMassVelocityDelta)
   {
      centerOfMassVelocity.set(centerOfMassVelocityPort.getData());
      centerOfMassVelocity.add(centerOfMassVelocityDelta);
      centerOfMassVelocityPort.setData(centerOfMassVelocity);
   }
}
