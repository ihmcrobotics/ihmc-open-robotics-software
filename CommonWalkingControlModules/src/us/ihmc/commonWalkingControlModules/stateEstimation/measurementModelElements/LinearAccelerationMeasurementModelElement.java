package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LinearAccelerationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector> angularAccelerationPort;

   private final ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort;

   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;

   private final RigidBody estimationLink;
   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // intermediate result stuff:
   private final Matrix3d rotationFromWorldToEstimation = new Matrix3d();
   private final Matrix3d rotationFromEstimationToMeasurement = new Matrix3d();
   private final FrameVector angularVelocityOfMeasurementWithRespectToEstimation = new FrameVector(ReferenceFrame.getWorldFrame());
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final Matrix3d tempMatrix2 = new Matrix3d();
   private final Vector3d tempVector = new Vector3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist twistOfMeasurementFrameWithRespectToEstimation = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector gravitationalAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   public LinearAccelerationMeasurementModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
           ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowOutputPort<FrameVector> angularAccelerationPort, ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort,
           TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator, RigidBody measurementLink,
           ReferenceFrame measurementFrame, RigidBody estimationLink, ReferenceFrame estimationFrame, double gZ)
   {
      super(SIZE, name, registry);
      MathTools.checkIfInRange(gZ, 0.0, Double.NEGATIVE_INFINITY);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;

      this.linearAccelerationMeasurementInputPort = linearAccelerationMeasurementInputPort;

      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;

      this.estimationLink = estimationLink;
      this.estimationFrame = estimationFrame;

      gravitationalAcceleration.setZ(gZ);

      outputMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(centerOfMassAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
   }

   public void computeMatrixBlocks()
   {
      // R_{w}^{p}
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tempTransform, estimationFrame);
      tempTransform.get(rotationFromWorldToEstimation);

      // R_{p}^{i}
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(rotationFromEstimationToMeasurement);

      // \omega_{i}^{p,p}
      twistCalculator.packRelativeTwist(twistOfMeasurementFrameWithRespectToEstimation, estimationLink, measurementLink);
      twistOfMeasurementFrameWithRespectToEstimation.packAngularPart(angularVelocityOfMeasurementWithRespectToEstimation);
      angularVelocityOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);

      computeCenterOfMassVelocityBlock(rotationFromEstimationToMeasurement, rotationFromWorldToEstimation, angularVelocityOfMeasurementWithRespectToEstimation);
      computeCenterOfMassAccelerationBlock();
      computeOrientationBlock(rotationFromEstimationToMeasurement, angularVelocityOfMeasurementWithRespectToEstimation);
      computeAngularVelocityBlock(rotationFromEstimationToMeasurement, twistOfMeasurementFrameWithRespectToEstimation);
      computeAngularAccelerationBlock(rotationFromEstimationToMeasurement);
   }

   private void computeOrientationBlock(Matrix3d rotationFromEstimationToMeasurement, FrameVector angularVelocityOfMeasurementWithRespectToEstimation)
   {
      // \tilde{R_{w}^{p} (\ddot{r} + g)}
      tempFrameVector.setAndChangeFrame(centerOfMassAccelerationPort.getData());
      tempFrameVector.changeFrame(gravitationalAcceleration.getReferenceFrame());
      tempFrameVector.sub(gravitationalAcceleration);
      tempFrameVector.changeFrame(estimationFrame);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());

      // \omega + \omega_{i}^{p,p}
      angularVelocityPort.getData().checkReferenceFrameMatch(estimationFrame);
      angularVelocityOfMeasurementWithRespectToEstimation.checkReferenceFrameMatch(estimationFrame);
      angularVelocityPort.getData().getVector(tempVector);
      tempVector.add(angularVelocityOfMeasurementWithRespectToEstimation.getVector());

      // \tilde{\omega + \omega_{i}^{p,p}} \tilde{R_{w}^{p} \dot{r}}
      tempFrameVector.setAndChangeFrame(centerOfMassVelocityPort.getData());
      tempFrameVector.changeFrame(estimationFrame);
      setTildeTimesTilde(tempMatrix2, tempVector, tempFrameVector.getVector());
      tempMatrix.add(tempMatrix2);

      // premultiply R_{p}^{i}
      tempMatrix.mul(rotationFromEstimationToMeasurement, tempMatrix);

      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
   }

   private void computeAngularVelocityBlock(Matrix3d rotationFromEstimationToMeasurement, Twist twistOfMeasurementFrameWithRespectToEstimation)
   {
      // TODO: garbage
      twistOfMeasurementFrameWithRespectToEstimation.changeFrame(estimationFrame);
      Vector3d omega = twistOfMeasurementFrameWithRespectToEstimation.getAngularPartCopy();
      Vector3d v = twistOfMeasurementFrameWithRespectToEstimation.getLinearPartCopy();

      FramePoint p = new FramePoint(measurementFrame);
      p.changeFrame(estimationFrame);

      FramePoint rP = new FramePoint(centerOfMassPositionPort.getData());
      rP.changeFrame(estimationFrame);

      FrameVector rd = new FrameVector(centerOfMassVelocityPort.getData());
      rd.changeFrame(estimationFrame);

      // \dot{r}^{p} = R_{w}^{p} \dot{r} - \tilde{\omega}r^{p} - v_{p}^{p,w}
      FrameVector rPd = new FrameVector(rd);
      twistOfMeasurementFrameWithRespectToEstimation.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      rPd.sub(tempFrameVector);
      twistOfMeasurementFrameWithRespectToEstimation.packLinearPart(tempFrameVector);
      rPd.sub(tempFrameVector);

      tempMatrix.setZero();

      // \tilde{p}_{i}^{p} \tilde{\omega}_{i}^{p,p}
      setTildeTimesTilde(tempMatrix2, p.getPoint(), omega);
      tempMatrix.add(tempMatrix2);

      // -\tilde{v}_{i}^{p,p}
      MatrixTools.toTildeForm(tempMatrix2, v);
      tempMatrix.sub(tempMatrix2);

      // -\tilde{\omega}_{i}^{p,p} \tilde{r}^{p}
      setTildeTimesTilde(tempMatrix2, omega, rP.getPoint());
      tempMatrix.sub(tempMatrix2);

      // \tilde{R_{w}^{p} \dot{r}}
      MatrixTools.toTildeForm(tempMatrix2, rd.getVector());
      tempMatrix.add(tempMatrix2);

      // \tilde{\dot{r}}^{p}
      MatrixTools.toTildeForm(tempMatrix2, rPd.getVector());
      tempMatrix.add(tempMatrix2);

      // premultiply R_{p}^{i}
      tempMatrix.mul(rotationFromEstimationToMeasurement, tempMatrix);

      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularVelocityPort));
   }

   private void computeCenterOfMassVelocityBlock(Matrix3d rotationFromEstimationToMeasurement, Matrix3d rotationFromWorldToEstimation,
           FrameVector angularVelocityOfMeasurementFrameWithRespectToEstimation)
   {
      // \omega + \omega_{i}^{p,p}
      angularVelocityPort.getData().checkReferenceFrameMatch(estimationFrame);
      angularVelocityOfMeasurementFrameWithRespectToEstimation.checkReferenceFrameMatch(estimationFrame);
      angularVelocityPort.getData().getVector(tempVector);
      tempVector.add(angularVelocityOfMeasurementFrameWithRespectToEstimation.getVector());

      // -\tilde{\omega + \omega_{i}^{p,p}}
      tempVector.negate();
      MatrixTools.toTildeForm(tempMatrix, tempVector);

      // -R_{p}^{i} \tilde{\omega + \omega_{i}^{p,p}}
      tempMatrix.mul(rotationFromEstimationToMeasurement, tempMatrix);

      // -R_{p}^{i} \tilde{\omega + \omega_{i}^{p,p}} R_{w}^{p}
      tempMatrix.mul(tempMatrix, rotationFromWorldToEstimation);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(centerOfMassVelocityPort));
   }

   private void computeCenterOfMassAccelerationBlock()
   {
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(centerOfMassAccelerationPort));
   }

   private void computeAngularAccelerationBlock(Matrix3d rotationFromEstimationToMeasurement)
   {
      // r^{p}
      tempFramePoint.setAndChangeFrame(centerOfMassPositionPort.getData());
      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.getVector(tempVector);

      // r^{p} - p_{i}^{p}
      tempFramePoint.setToZero(measurementFrame);
      tempFramePoint.changeFrame(estimationFrame);
      tempVector.sub(tempFramePoint.getPoint());

      // \tilde{r^{p} - p_{i}^{p}}
      MatrixTools.toTildeForm(tempMatrix, tempVector);

      // R_{p}^{i} \tilde{r^{p} - p_{i}^{p}}
      tempMatrix.mul(rotationFromEstimationToMeasurement, tempMatrix);

      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularAccelerationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      tempFramePoint.setToZero(measurementFrame);
      spatialAccelerationCalculator.packLinearAccelerationOfBodyFixedPoint(tempFrameVector, measurementLink, tempFramePoint);
      tempFrameVector.changeFrame(measurementFrame);
      tempVector.set(linearAccelerationMeasurementInputPort.getData());
      tempVector.sub(tempFrameVector.getVector());

      MatrixTools.insertTuple3dIntoEJMLVector(tempVector, residual, 0);

      return residual;
   }

   /*
    * M = \tilde{a} * \tilde{b}
    */
   private static void setTildeTimesTilde(Matrix3d M, Tuple3d a, Tuple3d b)
   {
      double axbx = a.x * b.x;
      double ayby = a.y * b.y;
      double azbz = a.z * b.z;

      M.m00 = -azbz - ayby;
      M.m01 = a.y * b.x;
      M.m02 = a.z * b.x;

      M.m10 = a.x * b.y;
      M.m11 = -axbx - azbz;
      M.m12 = a.z * b.y;

      M.m20 = a.x * b.z;
      M.m21 = a.y * b.z;
      M.m22 = -axbx - ayby;
   }
}
