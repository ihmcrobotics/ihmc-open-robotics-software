package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public class PointVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   public static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort;
   
   private final boolean assumePerfectIMU;
   private final ControlFlowOutputPort<FrameQuaternion> orientationPort;
   private final ControlFlowOutputPort<FrameVector3D> angularVelocityPort;

   private final ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final RotationMatrix rotationFromEstimationToWorld = new RotationMatrix();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final Twist tempTwist = new Twist();
   private final FrameVector3D tempFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempFrameVector2 = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D residualVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   
   private final AfterJointReferenceFrameNameMap referenceFrameNameMap;
   private final RigidBodyToIndexMap estimatorRigidBodyToIndexMap;
   

   public PointVelocityMeasurementModelElement(String name, ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort,
         ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameQuaternion> orientationPort, ControlFlowOutputPort<FrameVector3D> angularVelocityPort, ReferenceFrame estimationFrame,
         ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort, AfterJointReferenceFrameNameMap referenceFrameNameMap,
         RigidBodyToIndexMap estimatorRigidBodyToIndexMap, boolean assumePerfectIMU, YoVariableRegistry registry)
   {
      super(SIZE, name, registry);

      this.assumePerfectIMU = assumePerfectIMU;
      
      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;

      this.pointVelocityMeasurementInputPort = pointVelocityMeasurementInputPort;
      this.pointVelocityMeasurementInputPort.setData(new PointVelocityDataObject());

      this.estimationFrame = estimationFrame;
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      
      this.referenceFrameNameMap = referenceFrameNameMap;
      this.estimatorRigidBodyToIndexMap = estimatorRigidBodyToIndexMap;

      if (assumePerfectIMU)
         initialize(SIZE, centerOfMassPositionPort, centerOfMassVelocityPort);
      else
         initialize(SIZE, centerOfMassPositionPort, centerOfMassVelocityPort, orientationPort, angularVelocityPort);
      
      computeCenterOfMassVelocityStateOutputBlock();
   }

   public ControlFlowInputPort<PointVelocityDataObject> getPointVelocityMeasurementInputPort()
   {
      return pointVelocityMeasurementInputPort;
   }

   public void computeMatrixBlocks()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.getRotation(rotationFromEstimationToWorld);
      
      if (!assumePerfectIMU)
      {
         computeOrientationStateOutputBlock(rotationFromEstimationToWorld);
         computeAngularVelocityStateOutputBlock(rotationFromEstimationToWorld);
      }
   }

   private void computeCenterOfMassVelocityStateOutputBlock()
   {
      CommonOps.setIdentity(getOutputMatrixBlock(centerOfMassVelocityPort));
   }

   private final FramePoint3D tempCenterOfMassPosition = new FramePoint3D();
   private void computeAngularVelocityStateOutputBlock(RotationMatrix rotationFromPelvisToWorld)
   {
      tempCenterOfMassPosition.setIncludingFrame(centerOfMassPositionPort.getData());
      tempCenterOfMassPosition.changeFrame(estimationFrame);

      ReferenceFrame referenceFrame = referenceFrameNameMap.getFrameByName(pointVelocityMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, pointVelocityMeasurementInputPort.getData().getMeasurementPointInBodyFrame());
      
      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(tempCenterOfMassPosition);
      tempFramePoint.scale(-1.0);

      tempMatrix.setToTildeForm(tempFramePoint.getPoint());
      tempMatrix.preMultiply(rotationFromPelvisToWorld);
      tempMatrix.get(getOutputMatrixBlock(angularVelocityPort));
   }

   private final FrameVector3D tempCenterOfMassVelocity = new FrameVector3D();
   private void computeOrientationStateOutputBlock(RotationMatrix rotationFromPelvisToWorld)
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(estimationFrame);

      tempCenterOfMassVelocity.setIncludingFrame(centerOfMassVelocityPort.getData());
      tempCenterOfMassVelocity.changeFrame(estimationFrame);
      tempFrameVector.sub(tempCenterOfMassVelocity);
      tempFrameVector.scale(-1.0);

      tempMatrix.setToTildeForm(tempFrameVector.getVector());
      tempMatrix.preMultiply(rotationFromPelvisToWorld);
      tempMatrix.get(getOutputMatrixBlock(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
      tempFrameVector2.setIncludingFrame(ReferenceFrame.getWorldFrame(), pointVelocityMeasurementInputPort.getData().getVelocityOfMeasurementPointInWorldFrame());

      residualVector.setIncludingFrame(tempFrameVector2);
      residualVector.sub(tempFrameVector);

      residualVector.getVector().get(residual);
      return residual;
   }

   private void computeVelocityOfStationaryPoint(FrameVector3D stationaryPointVelocityToPack)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      RigidBody stationaryPointLink = estimatorRigidBodyToIndexMap.getRigidBodyByName(pointVelocityMeasurementInputPort.getData().getRigidBodyName());

      stationaryPointLink.getBodyFixedFrame().getTwistOfFrame(tempTwist);
      tempTwist.changeFrame(tempTwist.getBaseFrame());
      ReferenceFrame referenceFrame = referenceFrameNameMap.getFrameByName(pointVelocityMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, pointVelocityMeasurementInputPort.getData().getMeasurementPointInBodyFrame());
      tempFramePoint.changeFrame(tempTwist.getBaseFrame());
      tempTwist.getLinearVelocityOfPointFixedInBodyFrame(stationaryPointVelocityToPack, tempFramePoint);
   }

}
