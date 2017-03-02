package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class LinearAccelerationMeasurementModelJacobianAssembler
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;
   private final ReferenceFrame estimationFrame;

   private final RotationMatrix rotationFromWorldToMeasurement = new RotationMatrix();

   private final Matrix3D omegaTilde = new Matrix3D();
   private final Matrix3D vTilde = new Matrix3D();
   private final Matrix3D pTilde = new Matrix3D();
   private final Matrix3D omegadTilde = new Matrix3D();
   private final Matrix3D zTildeRMP = new Matrix3D();

   private final Vector3D omega = new Vector3D();
   private final Vector3D v = new Vector3D();
   private final FramePoint p = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3D omegad = new Vector3D();

   private final Twist twistOfMeasurementLink = new Twist();
   private final SpatialAccelerationVector spatialAccelerationOfMeasurementLink = new SpatialAccelerationVector();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final Matrix3D tempMatrix2 = new Matrix3D();

   public LinearAccelerationMeasurementModelJacobianAssembler(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
                                                              RigidBody measurementLink, ReferenceFrame measurementFrame, ReferenceFrame estimationFrame)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;
      this.estimationFrame = estimationFrame;
   }

   public void preCompute(Vector3D unbiasedEstimatedMeasurement)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      RigidBody elevator = spatialAccelerationCalculator.getRootBody();
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();

      // T, Td
      twistCalculator.getRelativeTwist(twistOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationCalculator.getRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationOfMeasurementLink.changeFrame(elevatorFrame, twistOfMeasurementLink, twistOfMeasurementLink);
      twistOfMeasurementLink.changeFrame(elevatorFrame);

      // \tilde{\omega}, \tilde{v}
      twistOfMeasurementLink.getAngularPart(omega);
      omegaTilde.setToTildeForm(omega);
      twistOfMeasurementLink.getLinearPart(v);
      vTilde.setToTildeForm(v);

      // \tilde{p}
      p.setToZero(measurementFrame);
      p.changeFrame(elevatorFrame);
      pTilde.setToTildeForm(p.getPoint());

      // \tilde{\omegad}
      spatialAccelerationOfMeasurementLink.getAngularPart(omegad);
      omegadTilde.setToTildeForm(omegad);

      // rotation matrix
      elevatorFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(rotationFromWorldToMeasurement);

      // z
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(tempMatrix);
      zTildeRMP.setToTildeForm(unbiasedEstimatedMeasurement);
      zTildeRMP.multiply(tempMatrix);
   }

   public void assembleMeasurementJacobian(Matrix3D ret, Matrix3D jPhi, Matrix3D jOmega, Matrix3D jV, Matrix3D jOmegad, Matrix3D jVd, Matrix3D jP)
   {
      ret.setToZero();

      if (jP != null)
      {
         tempMatrix.set(omegaTilde);
         tempMatrix.multiply(omegaTilde);
         tempMatrix.add(omegadTilde);
         tempMatrix.multiply(jP);
         ret.add(tempMatrix);
      }

      if (jOmega != null)
      {
         tempMatrix.set(omegaTilde);
         tempMatrix.multiply(pTilde);
         tempMatrix.scale(2.0);
         tempMatrix2.set(pTilde);
         tempMatrix2.multiply(omegaTilde);
         tempMatrix.sub(tempMatrix2);
         tempMatrix.add(vTilde);
         tempMatrix.multiply(jOmega);
         ret.sub(tempMatrix);
      }

      if (jV != null)
      {
         tempMatrix.set(omegaTilde);
         tempMatrix.multiply(jV);
         ret.add(tempMatrix);
      }

      if (jOmegad != null)
      {
         tempMatrix.set(pTilde);
         tempMatrix.multiply(jOmegad);
         ret.sub(tempMatrix);
      }

      if (jVd != null)
      {
         ret.add(jVd);
      }

      ret.preMultiply(rotationFromWorldToMeasurement);

      if (jPhi != null)
      {
         tempMatrix.set(zTildeRMP);
         tempMatrix.multiply(jPhi);
         ret.add(tempMatrix);
      }
   }
}
