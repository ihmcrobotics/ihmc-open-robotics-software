package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class OrientationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
   private final ControlFlowInputPort<Matrix3d> orientationMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame measurementFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // temp stuff:
   private final FrameOrientation orientationOfMeasurementFrameInEstimationFrame = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final Matrix3d tempMatrix3d = new Matrix3d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   
   private final Quat4d estimatedOrientationQuaternion = new Quat4d();
   private final Quat4d measurmentFrameToEstimationFrame = new Quat4d();
   
   private final Quat4d orientationResidual = new Quat4d();
   private final Quat4d actualOrientationMeasurement = new Quat4d();
   private final Quat4d estimatedOrientationMeasurement = new Quat4d();
   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();
   private final Vector3d rotationVectorResidual = new Vector3d();

   public OrientationMeasurementModelElement(ControlFlowOutputPort<FrameOrientation> orientationStatePort,
           ControlFlowInputPort<Matrix3d> orientationMeasurementInputPort, ReferenceFrame estimationFrame, ReferenceFrame measurementFrame, String name,
           YoVariableRegistry registry)
   {
      super(SIZE, name, registry);
      this.orientationStatePort = orientationStatePort;
      this.orientationMeasurementInputPort = orientationMeasurementInputPort;
      this.estimationFrame = estimationFrame;
      this.measurementFrame = measurementFrame;

      initialize(SIZE, orientationStatePort);
      
      computeOrientationStateOutputBlock();
   }

   private void computeOrientationStateOutputBlock()
   {
      DenseMatrix64F orientationStateOutputBlock = getOutputMatrixBlock(orientationStatePort);
      CommonOps.setIdentity(orientationStateOutputBlock);
   }

   public void computeMatrixBlocks()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(tempMatrix3d);
      
      DenseMatrix64F orientationStateOutputBlock = getOutputMatrixBlock(orientationStatePort);  
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix3d, orientationStateOutputBlock);
   }

   private final FrameOrientation tempMeasuredOrientationFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   
   public DenseMatrix64F computeResidual()
   {
      orientationStatePort.getData().getQuaternion(estimatedOrientationQuaternion);

      orientationOfMeasurementFrameInEstimationFrame.setToZero(measurementFrame);
      orientationOfMeasurementFrameInEstimationFrame.changeFrame(estimationFrame);
      orientationOfMeasurementFrameInEstimationFrame.getQuaternion(measurmentFrameToEstimationFrame);
      
      // Compute orientationResidual as a quaternion
      tempMeasuredOrientationFrameOrientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), orientationMeasurementInputPort.getData());
      tempMeasuredOrientationFrameOrientation.getQuaternion(actualOrientationMeasurement);
     
      // Compute the estimated measurement
      estimatedOrientationMeasurement.set(estimatedOrientationQuaternion);
      estimatedOrientationMeasurement.mul(measurmentFrameToEstimationFrame);
      
      // Compare the estimated and actual measurement to get the residual as a quaternion.
      orientationResidual.set(estimatedOrientationMeasurement);
      orientationResidual.inverse();
      orientationResidual.mul(actualOrientationMeasurement);
       
      // Convert to rotationVectorResidual
      tempAxisAngle.set(0.0, 0.0, 0.0, 0.0);    // necessary because set(Quat4d) may not actually set anything if magnitude is small!
      tempAxisAngle.set(orientationResidual);
      rotationVectorResidual.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      double angle = AngleTools.trimAngleMinusPiToPi(tempAxisAngle.getAngle());
      rotationVectorResidual.scale(angle);
      MatrixTools.insertTuple3dIntoEJMLVector(rotationVectorResidual, residual, 0);

      return residual;
   }
}
