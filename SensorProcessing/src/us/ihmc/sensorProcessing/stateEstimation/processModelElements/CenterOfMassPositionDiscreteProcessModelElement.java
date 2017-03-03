package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassPositionDiscreteProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;

   // temp stuff
   private final FramePoint centerOfMassPosition;
   private final FrameVector centerOfMassPositionDelta;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public CenterOfMassPositionDiscreteProcessModelElement(double deltaT, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
         ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, String name, YoVariableRegistry registry)
   {
      super(centerOfMassPositionPort, TimeDomain.DISCRETE, false, SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassPosition = new FramePoint(worldFrame);
      this.centerOfMassPositionDelta = new FrameVector(ReferenceFrame.getWorldFrame());

      stateMatrixBlocks.put(centerOfMassPositionPort, new DenseMatrix64F(SIZE, SIZE));
      stateMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      computeCenterOfMassPositionStateMatrixBlock();
      computeCenterOfMassVelocityStateMatrixBlock(deltaT);
   }

   private void computeCenterOfMassPositionStateMatrixBlock()
   {
      DenseMatrix64F stateMatrixBlock = stateMatrixBlocks.get(centerOfMassPositionPort);
      CommonOps.setIdentity(stateMatrixBlock);
   }

   private void computeCenterOfMassVelocityStateMatrixBlock(double deltaT)
   {
      // Euler integrator: x_n+1 = x_n + deltaT * x_dot_n
      DenseMatrix64F stateMatrixBlock = stateMatrixBlocks.get(centerOfMassVelocityPort);
      CommonOps.setIdentity(stateMatrixBlock);
      CommonOps.scale(deltaT, stateMatrixBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      FrameVector centerOfMassVelocity = centerOfMassVelocityPort.getData();
      centerOfMassVelocity.changeFrame(worldFrame);
      centerOfMassPositionDelta.set(centerOfMassVelocity);
      centerOfMassPositionDelta.scale(dt);

      updateCenterOfMassPosition(centerOfMassPositionDelta);
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(centerOfMassPositionDelta.getVector(), correction, 0);
      updateCenterOfMassPosition(centerOfMassPositionDelta);
   }

   private void updateCenterOfMassPosition(FrameVector centerOfMassPositionDelta)
   {
      centerOfMassPosition.set(centerOfMassPositionPort.getData());
      centerOfMassPosition.add(centerOfMassPositionDelta);
      centerOfMassPositionPort.setData(centerOfMassPosition);
   }
}
