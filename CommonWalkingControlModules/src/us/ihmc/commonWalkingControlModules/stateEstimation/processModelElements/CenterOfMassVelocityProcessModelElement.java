package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CenterOfMassVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowInputPort<FrameVector> centerOfMassAccelerationPort;

   // temp stuff
   private final FrameVector centerOfMassVelocity;
   private final FrameVector centerOfMassVelocityDelta;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public CenterOfMassVelocityProcessModelElement(ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
           ControlFlowInputPort<FrameVector> centerOfMassAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(SIZE, 0, 1, name, registry);
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;
      this.centerOfMassVelocity = new FrameVector(worldFrame);
      this.centerOfMassVelocityDelta = new FrameVector(ReferenceFrame.getWorldFrame());

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
         FrameVector centerOfMassAcceleration = centerOfMassAccelerationPort.getData();
         centerOfMassAcceleration.changeFrame(worldFrame);
         centerOfMassVelocityDelta.set(centerOfMassAcceleration);
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
