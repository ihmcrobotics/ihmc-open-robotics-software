package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;


import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class AngularVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ReferenceFrame estimationFrame;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowInputPort<FrameVector> angularAccelerationPort;

   // temp stuff
   private final Vector3d angularVelocity = new Vector3d();
   private final Vector3d angularVelocityDelta = new Vector3d();

   public AngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowInputPort<FrameVector> angularAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(SIZE, 0, 1, name, registry);
      this.estimationFrame = estimationFrame;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;

      inputMatrixBlocks.put(angularAccelerationPort, new DenseMatrix64F(SIZE, SIZE));

      computeAngularAccelerationInputMatrixBlock();
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
         angularAcceleration.getVector(angularVelocityDelta);
         angularVelocityDelta.scale(dt);

         updateAngularVelocity(angularVelocityDelta);
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(angularVelocityDelta, correction, 0);
      updateAngularVelocity(angularVelocityDelta);
   }

   private void updateAngularVelocity(Vector3d angularVelocityDelta)
   {
      angularVelocityPort.getData().checkReferenceFrameMatch(estimationFrame);
      angularVelocityPort.getData().getVector(angularVelocity);
      angularVelocity.add(angularVelocityDelta);
      angularVelocityPort.getData().set(angularVelocity);
   }
}
