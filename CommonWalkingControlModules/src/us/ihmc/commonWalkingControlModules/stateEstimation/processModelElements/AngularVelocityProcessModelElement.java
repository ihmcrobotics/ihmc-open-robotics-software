package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.TimeDomain;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class AngularVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ReferenceFrame estimationFrame;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowInputPort<FrameVector> angularAccelerationPort;

   // temp stuff
   private final FrameVector angularVelocity;
   private final FrameVector angularVelocityDelta;

   public AngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowInputPort<FrameVector> angularAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(angularVelocityPort, TimeDomain.CONTINUOUS, false, SIZE, name, registry);
      this.estimationFrame = estimationFrame;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;
      this.angularVelocity = new FrameVector(estimationFrame);
      this.angularVelocityDelta = new FrameVector(estimationFrame);

      if (angularAccelerationPort != null)
      {
         inputMatrixBlocks.put(angularAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
         computeAngularAccelerationInputMatrixBlock();
      }
   }

   private void computeAngularAccelerationInputMatrixBlock()
   {
      CommonOps.setIdentity(inputMatrixBlocks.get(angularAccelerationPort));
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      if (angularAccelerationPort != null)
      {
         FrameVector angularAcceleration = angularAccelerationPort.getData();
         angularAcceleration.changeFrame(estimationFrame);
         angularVelocityDelta.set(angularAcceleration);
         angularVelocityDelta.scale(dt);

         updateAngularVelocity(angularVelocityDelta);
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(angularVelocityDelta.getVector(), correction, 0);
      updateAngularVelocity(angularVelocityDelta);
   }

   private void updateAngularVelocity(FrameVector angularVelocityDelta)
   {
      angularVelocity.set(angularVelocityPort.getData());
      angularVelocity.add(angularVelocityDelta);
      angularVelocityPort.setData(angularVelocity);
   }
}
