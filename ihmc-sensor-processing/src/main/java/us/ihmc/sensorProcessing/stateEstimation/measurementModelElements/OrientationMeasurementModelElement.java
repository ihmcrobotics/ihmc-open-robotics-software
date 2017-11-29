package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AngleTools;

public class OrientationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameQuaternion> orientationStatePort;
   private final ControlFlowInputPort<RotationMatrix> orientationMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame measurementFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // temp stuff:
   private final FrameQuaternion orientationOfMeasurementFrameInEstimationFrame = new FrameQuaternion(ReferenceFrame.getWorldFrame());
   private final RotationMatrix tempMatrix3d = new RotationMatrix();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   
   private final Quaternion estimatedOrientationQuaternion = new Quaternion();
   private final Quaternion measurmentFrameToEstimationFrame = new Quaternion();
   
   private final Quaternion orientationResidual = new Quaternion();
   private final Quaternion actualOrientationMeasurement = new Quaternion();
   private final Quaternion estimatedOrientationMeasurement = new Quaternion();
   private final AxisAngle tempAxisAngle = new AxisAngle();
   private final Vector3D rotationVectorResidual = new Vector3D();

   public OrientationMeasurementModelElement(ControlFlowOutputPort<FrameQuaternion> orientationStatePort,
           ControlFlowInputPort<RotationMatrix> orientationMeasurementInputPort, ReferenceFrame estimationFrame, ReferenceFrame measurementFrame, String name,
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
      tempTransform.getRotation(tempMatrix3d);
      
      DenseMatrix64F orientationStateOutputBlock = getOutputMatrixBlock(orientationStatePort);  
      tempMatrix3d.get(orientationStateOutputBlock);
   }

   private final FrameQuaternion tempMeasuredOrientationFrameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
   
   public DenseMatrix64F computeResidual()
   {
      estimatedOrientationQuaternion.set(orientationStatePort.getData());

      orientationOfMeasurementFrameInEstimationFrame.setToZero(measurementFrame);
      orientationOfMeasurementFrameInEstimationFrame.changeFrame(estimationFrame);
      measurmentFrameToEstimationFrame.set(orientationOfMeasurementFrameInEstimationFrame);
      
      // Compute orientationResidual as a quaternion
      tempMeasuredOrientationFrameOrientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), orientationMeasurementInputPort.getData());
      actualOrientationMeasurement.set(tempMeasuredOrientationFrameOrientation);
     
      // Compute the estimated measurement
      estimatedOrientationMeasurement.set(estimatedOrientationQuaternion);
      estimatedOrientationMeasurement.multiply(measurmentFrameToEstimationFrame);
      
      // Compare the estimated and actual measurement to get the residual as a quaternion.
      orientationResidual.set(estimatedOrientationMeasurement);
      orientationResidual.inverse();
      orientationResidual.multiply(actualOrientationMeasurement);
       
      // Convert to rotationVectorResidual
      tempAxisAngle.set(0.0, 0.0, 0.0, 0.0);    // necessary because set(Quat4d) may not actually set anything if magnitude is small!
      tempAxisAngle.set(orientationResidual);
      rotationVectorResidual.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      double angle = AngleTools.trimAngleMinusPiToPi(tempAxisAngle.getAngle());
      rotationVectorResidual.scale(angle);
      rotationVectorResidual.get(residual);

      return residual;
   }
}
