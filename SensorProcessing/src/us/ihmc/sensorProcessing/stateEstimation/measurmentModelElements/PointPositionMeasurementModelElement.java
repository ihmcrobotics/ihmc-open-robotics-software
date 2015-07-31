package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.utilities.math.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class PointPositionMeasurementModelElement extends AbstractMeasurementModelElement
{
   public static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort;

   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d residualVector = new Vector3d();

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
      tempTransform.get(rotationFromEstimationToWorld);

      tempCenterOfMassPosition.setIncludingFrame(centerOfMassPositionPort.getData());
      tempCenterOfMassPosition.changeFrame(estimationFrame);

      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, pointPositionMeasurementInputPort.getData().getMeasurementPointInBodyFrame());

      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(tempCenterOfMassPosition);
      tempFramePoint.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromEstimationToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      PointPositionDataObject data = pointPositionMeasurementInputPort.getData();
      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setIncludingFrame(referenceFrame, data.getMeasurementPointInBodyFrame());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

      residualVector.set(data.getMeasurementPointInWorldFrame());
      residualVector.sub(tempFramePoint.getPoint());

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector, residual, 0);

      return residual;
   }

   public ControlFlowInputPort<PointPositionDataObject> getPointPositionMeasurementInputPort()
   {
      return pointPositionMeasurementInputPort;
   }
}
