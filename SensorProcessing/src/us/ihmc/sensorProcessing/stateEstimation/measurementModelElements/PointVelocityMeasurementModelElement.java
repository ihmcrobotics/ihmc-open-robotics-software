package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public class PointVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   public static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   
   private final boolean assumePerfectIMU;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;

   private final ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final RotationMatrix rotationFromEstimationToWorld = new RotationMatrix();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist tempTwist = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempFrameVector2 = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector residualVector = new FrameVector(ReferenceFrame.getWorldFrame());

   
   private final AfterJointReferenceFrameNameMap referenceFrameNameMap;
   private final RigidBodyToIndexMap estimatorRigidBodyToIndexMap;
   

   public PointVelocityMeasurementModelElement(String name, ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort, ReferenceFrame estimationFrame,
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

   private final FramePoint tempCenterOfMassPosition = new FramePoint();
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

   private final FrameVector tempCenterOfMassVelocity = new FrameVector();
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

   private void computeVelocityOfStationaryPoint(FrameVector stationaryPointVelocityToPack)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      RigidBody stationaryPointLink = estimatorRigidBodyToIndexMap.getRigidBodyByName(pointVelocityMeasurementInputPort.getData().getRigidBodyName());

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      try
      {
         twistCalculator.getTwistOfBody(tempTwist, stationaryPointLink);
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }
      tempTwist.changeFrame(tempTwist.getBaseFrame());
      ReferenceFrame referenceFrame = referenceFrameNameMap.getFrameByName(pointVelocityMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, pointVelocityMeasurementInputPort.getData().getMeasurementPointInBodyFrame());
      tempFramePoint.changeFrame(tempTwist.getBaseFrame());
      tempTwist.getLinearVelocityOfPointFixedInBodyFrame(stationaryPointVelocityToPack, tempFramePoint);
   }

}
