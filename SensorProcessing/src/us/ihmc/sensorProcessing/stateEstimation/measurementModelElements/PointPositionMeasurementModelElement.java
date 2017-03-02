package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;

public class PointPositionMeasurementModelElement extends AbstractMeasurementModelElement
{
   public static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort;

   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final RotationMatrix rotationFromEstimationToWorld = new RotationMatrix();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3D residualVector = new Vector3D();

   private final AfterJointReferenceFrameNameMap referenceFrameMap;

   private final boolean assumePerfectIMU;
   
   public PointPositionMeasurementModelElement(String name, ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameOrientation> orientationPort, ReferenceFrame estimationFrame,
         AfterJointReferenceFrameNameMap referenceFrameMap, boolean assumePerfectIMU, YoVariableRegistry registry)
   {
      super(SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      
      this.orientationPort = orientationPort;
      this.assumePerfectIMU = assumePerfectIMU;
      if (assumePerfectIMU && orientationPort != null) throw new RuntimeException();
      if (!assumePerfectIMU && orientationPort == null) throw new RuntimeException();
      
      this.pointPositionMeasurementInputPort = pointPositionMeasurementInputPort;

      this.estimationFrame = estimationFrame;
      this.referenceFrameMap = referenceFrameMap;

      if(assumePerfectIMU)
         initialize(SIZE, centerOfMassPositionPort);
      else
         initialize(SIZE, centerOfMassPositionPort, orientationPort);
      
      computeCenterOfMassPositionStateOutputBlock();
   }

   public void computeMatrixBlocks()
   {
      if (!assumePerfectIMU) computeOrientationStateOutputBlock();
   }

   private void computeCenterOfMassPositionStateOutputBlock()
   {
      CommonOps.setIdentity(getOutputMatrixBlock(centerOfMassPositionPort));
   }
   
   private final FramePoint tempCenterOfMassPosition = new FramePoint();
   
   private void computeOrientationStateOutputBlock()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.getRotation(rotationFromEstimationToWorld);

      tempCenterOfMassPosition.setIncludingFrame(centerOfMassPositionPort.getData());
      tempCenterOfMassPosition.changeFrame(estimationFrame);

      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, pointPositionMeasurementInputPort.getData().getMeasurementPointInBodyFrame());

      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(tempCenterOfMassPosition);
      tempFramePoint.scale(-1.0);

      tempMatrix.setToTildeForm(tempFramePoint.getPoint());
      tempMatrix.preMultiply(rotationFromEstimationToWorld);
      tempMatrix.get(getOutputMatrixBlock(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      PointPositionDataObject data = pointPositionMeasurementInputPort.getData();
      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, data.getMeasurementPointInBodyFrame());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

      residualVector.set(data.getMeasurementPointInWorldFrame());
      residualVector.sub(tempFramePoint.getPoint());

      residualVector.get(residual);

      return residual;
   }

   public ControlFlowInputPort<PointPositionDataObject> getPointPositionMeasurementInputPort()
   {
      return pointPositionMeasurementInputPort;
   }
}
