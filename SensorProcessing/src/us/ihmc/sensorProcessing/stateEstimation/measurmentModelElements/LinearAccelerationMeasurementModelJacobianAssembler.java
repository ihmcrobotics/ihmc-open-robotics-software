package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class LinearAccelerationMeasurementModelJacobianAssembler
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;
   private final ReferenceFrame estimationFrame;

   private final Matrix3d rotationFromWorldToMeasurement = new Matrix3d();

   private final Matrix3d omegaTilde = new Matrix3d();
   private final Matrix3d vTilde = new Matrix3d();
   private final Matrix3d pTilde = new Matrix3d();
   private final Matrix3d omegadTilde = new Matrix3d();
   private final Matrix3d zTildeRMP = new Matrix3d();

   private final Vector3d omega = new Vector3d();
   private final Vector3d v = new Vector3d();
   private final FramePoint p = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d omegad = new Vector3d();

   private final Twist twistOfMeasurementLink = new Twist();
   private final SpatialAccelerationVector spatialAccelerationOfMeasurementLink = new SpatialAccelerationVector();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final Matrix3d tempMatrix2 = new Matrix3d();

   public LinearAccelerationMeasurementModelJacobianAssembler(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           RigidBody measurementLink, ReferenceFrame measurementFrame, ReferenceFrame estimationFrame)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;
      this.estimationFrame = estimationFrame;
   }

   public void preCompute(Vector3d unbiasedEstimatedMeasurement)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      RigidBody elevator = spatialAccelerationCalculator.getRootBody();
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();

      // T, Td
      twistCalculator.packRelativeTwist(twistOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationCalculator.packRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationOfMeasurementLink.changeFrame(elevatorFrame, twistOfMeasurementLink, twistOfMeasurementLink);
      twistOfMeasurementLink.changeFrame(elevatorFrame);

      // \tilde{\omega}, \tilde{v}
      twistOfMeasurementLink.packAngularPart(omega);
      MatrixTools.toTildeForm(omegaTilde, omega);
      twistOfMeasurementLink.packLinearPart(v);
      MatrixTools.toTildeForm(vTilde, v);

      // \tilde{p}
      p.setToZero(measurementFrame);
      p.changeFrame(elevatorFrame);
      MatrixTools.toTildeForm(pTilde, p.getPoint());

      // \tilde{\omegad}
      spatialAccelerationOfMeasurementLink.packAngularPart(omegad);
      MatrixTools.toTildeForm(omegadTilde, omegad);

      // rotation matrix
      elevatorFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(rotationFromWorldToMeasurement);

      // z
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(tempMatrix);
      MatrixTools.toTildeForm(zTildeRMP, unbiasedEstimatedMeasurement);
      zTildeRMP.mul(tempMatrix);
   }

   public void assembleMeasurementJacobian(Matrix3d ret, Matrix3d jPhi, Matrix3d jOmega, Matrix3d jV, Matrix3d jOmegad, Matrix3d jVd, Matrix3d jP)
   {
      ret.setZero();

      if (jP != null)
      {
         tempMatrix.mul(omegaTilde, omegaTilde);
         tempMatrix.add(omegadTilde);
         tempMatrix.mul(jP);
         ret.add(tempMatrix);
      }

      if (jOmega != null)
      {
         tempMatrix.mul(omegaTilde, pTilde);
         tempMatrix.mul(2.0);
         tempMatrix2.mul(pTilde, omegaTilde);
         tempMatrix.sub(tempMatrix2);
         tempMatrix.add(vTilde);
         tempMatrix.mul(jOmega);
         ret.sub(tempMatrix);
      }

      if (jV != null)
      {
         tempMatrix.mul(omegaTilde, jV);
         ret.add(tempMatrix);
      }

      if (jOmegad != null)
      {
         tempMatrix.mul(pTilde, jOmegad);
         ret.sub(tempMatrix);
      }

      if (jVd != null)
      {
         ret.add(jVd);
      }

      ret.mul(rotationFromWorldToMeasurement, ret);

      if (jPhi != null)
      {
         tempMatrix.mul(zTildeRMP, jPhi);
         ret.add(tempMatrix);
      }
   }
}
