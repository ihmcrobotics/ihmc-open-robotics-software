package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;

public class StateEstimationDataFromController extends AbstractControlFlowElement
{
   private final ReferenceFrame angularAccelerationEstimationFrame;
   private final ReferenceFrame centerOfMassAccelerationFrame = ReferenceFrame.getWorldFrame();

   private final FrameVector desiredCenterOfMassAcceleration;
   private final FrameVector desiredAngularAcceleration;
   private final List<PointPositionDataObject> pointPositionDataObjects;
   private final List<PointVelocityDataObject> pointVelocityDataObjects;

   private final ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort;
   private final ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort;
   private final ControlFlowOutputPort<List<PointPositionDataObject>> pointPositionOutputPort;
   private final ControlFlowOutputPort<List<PointVelocityDataObject>> pointVelocityOutputPort;

   private int currentIndexInPointPositionList = 0;
   private int currentIndexInPointVelocityList = 0;

   public StateEstimationDataFromController(ReferenceFrame estimationFrame)
   {
      this.angularAccelerationEstimationFrame = estimationFrame;

      this.desiredCenterOfMassAccelerationOutputPort = new ControlFlowOutputPort<FrameVector>("desiredCoMAcceleration", this);
      registerOutputPort(desiredCenterOfMassAccelerationOutputPort);

      this.desiredAngularAccelerationOutputPort = new ControlFlowOutputPort<FrameVector>("desiredAngularAcceleration", this);
      registerOutputPort(desiredAngularAccelerationOutputPort);

      this.pointPositionOutputPort = new ControlFlowOutputPort<List<PointPositionDataObject>>("pointPosition", this);
      registerOutputPort(pointPositionOutputPort);
      
      this.pointVelocityOutputPort = new ControlFlowOutputPort<List<PointVelocityDataObject>>("pointVelocity", this);
      registerOutputPort(pointVelocityOutputPort);

      desiredCenterOfMassAcceleration = new FrameVector(centerOfMassAccelerationFrame);
      desiredAngularAcceleration = new FrameVector(angularAccelerationEstimationFrame);
      pointPositionDataObjects = new ArrayList<PointPositionDataObject>();
      pointVelocityDataObjects = new ArrayList<PointVelocityDataObject>();

      desiredCenterOfMassAccelerationOutputPort.setData(desiredCenterOfMassAcceleration);
      desiredAngularAccelerationOutputPort.setData(desiredAngularAcceleration);
      pointPositionOutputPort.setData(pointPositionDataObjects);
      pointVelocityOutputPort.setData(pointVelocityDataObjects);
   }
   
   public void setDesiredCenterOfMassAcceleration(FrameVector desiredCenterOfMassAcceleration)
   {
      checkReferenceFrameMatchByName(centerOfMassAccelerationFrame, desiredCenterOfMassAcceleration.getReferenceFrame());
      this.desiredCenterOfMassAcceleration.setIncludingFrame(centerOfMassAccelerationFrame, desiredCenterOfMassAcceleration.getVector());
      desiredCenterOfMassAccelerationOutputPort.setData(this.desiredCenterOfMassAcceleration);
   }

   public void setDesiredAngularAcceleration(FrameVector desiredAngularAcceleration)
   {
      checkReferenceFrameMatchByName(angularAccelerationEstimationFrame, desiredAngularAcceleration.getReferenceFrame());
      this.desiredAngularAcceleration.setIncludingFrame(angularAccelerationEstimationFrame, desiredAngularAcceleration.getVector());
      desiredAngularAccelerationOutputPort.setData(this.desiredAngularAcceleration);
   }
   
   private void setPointPositionMeasurements(List<PointPositionDataObject> pointPositionDataObjects)
   {
      extendPointPositionDataObjectList(pointPositionDataObjects.size());
      
      for (int i = 0; i < pointPositionDataObjects.size(); i++)
      {
         this.pointPositionDataObjects.get(i).set(pointPositionDataObjects.get(i));
      }
      
      pointPositionOutputPort.setData(this.pointPositionDataObjects);
   }
   
   private void setPointVelocityMeasurements(List<PointVelocityDataObject> pointVelocityDataObjects)
   {
      extendPointVelocityDataObjectList(pointVelocityDataObjects.size());
      
      for (int i = 0; i < pointVelocityDataObjects.size(); i++)
      {
         this.pointVelocityDataObjects.get(i).set(pointVelocityDataObjects.get(i));
      }
      
      pointVelocityOutputPort.setData(this.pointVelocityDataObjects);
   }

   public void clearDesiredAccelerations()
   {
      desiredCenterOfMassAcceleration.set(0.0, 0.0, 0.0);
      desiredAngularAcceleration.set(0.0, 0.0, 0.0);
   }
   
   public void clearPointPositionDataObjects()
   {
      currentIndexInPointPositionList = 0;
      
      for (PointPositionDataObject pointPositionDataObject : pointPositionDataObjects)
      {
         pointPositionDataObject.invalidatePointPosition();
      }
   }
   
   public void clearPointVelocityDataObjects()
   {
      currentIndexInPointVelocityList = 0;
      
      for (PointVelocityDataObject pointVelocityDataObject : pointVelocityDataObjects)
      {
         pointVelocityDataObject.invalidatePointVelocity();
      }
   }

   private FrameVector getDesiredAngularAcceleration()
   {
      return desiredAngularAcceleration;
   }

   // TODO make that private again (Sylvain)
   public FrameVector getDesiredCenterOfMassAcceleration()
   {
      return desiredCenterOfMassAcceleration;
   }

   public void updatePointPositionSensorData(PointPositionDataObject value)
   {
      setOrAddObjectToList(currentIndexInPointPositionList++, pointPositionDataObjects, value);
   }
   
   public void updatePointVelocitySensorData(PointVelocityDataObject value)
   {
      setOrAddObjectToList(currentIndexInPointVelocityList++, pointVelocityDataObjects, value);
   }

   private List<PointPositionDataObject> getPointPositionDataObjects()
   {
      return pointPositionDataObjects;
   }
   
   private List<PointVelocityDataObject> getPointVelocityDataObjects()
   {
      return pointVelocityDataObjects;
   }

   public void startComputation()
   {
   }

   public void waitUntilComputationIsDone()
   {
      // do nothing.
   }
   
   public void connectDesiredAccelerationPorts(ControlFlowGraph controlFlowGraph, StateEstimatorWithPorts orientationEstimatorWithPorts)
   {      
      ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort = orientationEstimatorWithPorts.getDesiredAngularAccelerationInputPort();
      ControlFlowInputPort<FrameVector> desiredCenterOfMassAccelerationInputPort = orientationEstimatorWithPorts.getDesiredCenterOfMassAccelerationInputPort();
      ControlFlowInputPort<List<PointPositionDataObject>> pointPositionInputPort = orientationEstimatorWithPorts.getPointPositionInputPort();
      ControlFlowInputPort<List<PointVelocityDataObject>> pointVelocityInputPort = orientationEstimatorWithPorts.getPointVelocityInputPort();
      
      controlFlowGraph.connectElements(desiredAngularAccelerationOutputPort, desiredAngularAccelerationInputPort);
      controlFlowGraph.connectElements(desiredCenterOfMassAccelerationOutputPort, desiredCenterOfMassAccelerationInputPort);

      controlFlowGraph.connectElements(pointPositionOutputPort, pointPositionInputPort);
      controlFlowGraph.connectElements(pointVelocityOutputPort, pointVelocityInputPort);
   }

   public void set(StateEstimationDataFromController stateEstimationDataFromController)
   {
      setDesiredCenterOfMassAcceleration(stateEstimationDataFromController.getDesiredCenterOfMassAcceleration());
      setDesiredAngularAcceleration(stateEstimationDataFromController.getDesiredAngularAcceleration());
      setPointPositionMeasurements(stateEstimationDataFromController.getPointPositionDataObjects());
      setPointVelocityMeasurements(stateEstimationDataFromController.getPointVelocityDataObjects());
   }

   private void extendPointPositionDataObjectList(int newSize)
   {
      for (int i = this.pointPositionDataObjects.size(); i < newSize; i++)
      {
         PointPositionDataObject pointPositionDataObject = new PointPositionDataObject();
         pointPositionDataObjects.add(pointPositionDataObject);
      }
   }

   private void extendPointVelocityDataObjectList(int newSize)
   {
      for (int i = this.pointVelocityDataObjects.size(); i < newSize; i++)
      {
         PointVelocityDataObject pointVelocityDataObject = new PointVelocityDataObject();
         pointVelocityDataObjects.add(pointVelocityDataObject);
      }
   }

   private static void checkReferenceFrameMatchByName(ReferenceFrame expected, ReferenceFrame actual)
   {
      if(!actual.getName().equals(expected.getName()))
      {
         throw new RuntimeException("Frame name does not match, expected: " + expected.getName() + ", actual: "
               + actual.getName());  
      }
   }

   private static <ObjectType> void setOrAddObjectToList(int i, List<ObjectType> list, ObjectType object)
   {
      if (i < list.size())
         list.set(i, object);
      else
         list.add(object);
   }
   
   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(desiredCenterOfMassAccelerationOutputPort.getData().getVector());
      checksum.update(desiredAngularAccelerationOutputPort.getData().getVector());
      
      List<PointPositionDataObject> pointPositions = pointPositionOutputPort.getData();
      for(int i = 0; i < pointPositions.size(); i++)
      {
         pointPositions.get(i).calculateChecksum(checksum);
      }
      
      List<PointVelocityDataObject> pointVelocities = pointVelocityOutputPort.getData();
      for(int i = 0; i < pointVelocities.size(); i++)
      {
         pointVelocities.get(i).calculateChecksum(checksum);
      }
   }

   public void initialize()
   {
//    empty
   }
}

