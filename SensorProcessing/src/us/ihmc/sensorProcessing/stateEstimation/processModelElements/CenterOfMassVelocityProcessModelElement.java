package us.ihmc.sensorProcessing.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort;
   private final ControlFlowInputPort<FrameVector3D> centerOfMassAccelerationPort;

   // temp stuff
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D centerOfMassVelocityDelta = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public CenterOfMassVelocityProcessModelElement(ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort,
           ControlFlowInputPort<FrameVector3D> centerOfMassAccelerationPort, String name, YoVariableRegistry registry)
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

   private void updateCenterOfMassVelocity(FrameVector3D centerOfMassVelocityDelta)
   {
      centerOfMassVelocity.set(centerOfMassVelocityPort.getData());
      centerOfMassVelocity.add(centerOfMassVelocityDelta);
      centerOfMassVelocityPort.setData(centerOfMassVelocity);
   }
}
