package us.ihmc.sensorProcessing.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowInputPort<FrameVector> centerOfMassAccelerationPort;

   // temp stuff
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);
   private final FrameVector centerOfMassVelocityDelta = new FrameVector(ReferenceFrame.getWorldFrame());

   public CenterOfMassVelocityProcessModelElement(ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
           ControlFlowInputPort<FrameVector> centerOfMassAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(centerOfMassVelocityPort, TimeDomain.CONTINUOUS, false, SIZE, name, registry);
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;

      if (centerOfMassAccelerationPort != null)
      {
         inputMatrixBlocks.put(centerOfMassAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
         computeCenterOfMassAccelerationInputMatrixBlock();
      }
   }

   private void computeCenterOfMassAccelerationInputMatrixBlock()
   {
      CommonOps.setIdentity(inputMatrixBlocks.get(centerOfMassAccelerationPort));
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      if (centerOfMassAccelerationPort != null)
      {
         centerOfMassVelocityDelta.set(centerOfMassAccelerationPort.getData());
         centerOfMassVelocityDelta.scale(dt);

         updateCenterOfMassVelocity(centerOfMassVelocityDelta);
      }
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
