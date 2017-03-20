package us.ihmc.sensorProcessing.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class OrientationProcessModelElement extends AbstractProcessModelElement
{
   static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D tempRotationVector = new Vector3D();
   private final AxisAngle tempAxisAngle = new AxisAngle();
   private final Matrix3D tempMatrix3d = new Matrix3D();

   private final Quaternion quaternionDelta = new Quaternion();
   private final Quaternion quaternion = new Quaternion();
   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

   public OrientationProcessModelElement(ControlFlowOutputPort<FrameVector> angularVelocityPort, ControlFlowOutputPort<FrameOrientation> orientationPort,
           String name, YoVariableRegistry registry)
   {
      super(orientationPort, TimeDomain.CONTINUOUS, true, SIZE, name, registry);
      this.angularVelocityPort = angularVelocityPort;
      this.orientationPort = orientationPort;

      stateMatrixBlocks.put(angularVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      stateMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));

      computeAngularVelocityStateMatrixBlock();
   }

   public void computeMatrixBlocks()
   {
      computeOrientationStateMatrixBlock();
   }

   private void computeOrientationStateMatrixBlock()
   {
      angularVelocityPort.getData().get(angularVelocity);
      angularVelocity.scale(-0.5);
      tempMatrix3d.setToTildeForm(angularVelocity);
      tempMatrix3d.get(stateMatrixBlocks.get(orientationPort));
   }

   private void computeAngularVelocityStateMatrixBlock()
   {
      CommonOps.setIdentity(stateMatrixBlocks.get(angularVelocityPort));
   }

   public void propagateState(double dt)
   {
      angularVelocityPort.getData().get(angularVelocity);
      orientationPort.getData().getQuaternion(quaternion);

      tempRotationVector.set(angularVelocity);
      tempRotationVector.scale(dt);

      tempAxisAngle.set(tempRotationVector);

      quaternionDelta.set(tempAxisAngle);
      quaternion.multiply(quaternionDelta);
      quaternion.normalize();    // the previous operation should preserve norm, so this might not be necessary every step
      orientation.set(quaternion);
      orientationPort.setData(orientation);
   }

   public void correctState(DenseMatrix64F correction)
   {
      orientationPort.getData().getQuaternion(quaternion);
      MatrixTools.extractTuple3dFromEJMLVector(tempRotationVector, correction, 0);
      tempAxisAngle.set(tempRotationVector);
      quaternionDelta.set(tempAxisAngle);
      quaternion.multiply(quaternionDelta);
      orientation.set(quaternion);
      orientationPort.setData(orientation);
   }
}
