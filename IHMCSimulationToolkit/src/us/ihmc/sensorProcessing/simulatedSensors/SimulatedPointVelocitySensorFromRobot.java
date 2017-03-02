package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.simulationconstructionset.KinematicPoint;

public class SimulatedPointVelocitySensorFromRobot extends SimulatedSensor<Tuple3DBasics>
{
   private final KinematicPoint kinematicPoint;
   
   private final RigidBody rigidBody;
   private final ReferenceFrame bodyFrame;
   
   private final Vector3D pointVelocity = new Vector3D();
   private final FramePoint measurementPointInBodyFrame = new FramePoint();
   private final FrameVector velocityOfMeasurementPointInWorldFrame = new FrameVector();
   
   private final PointVelocityDataObject pointVelocityDataObject = new PointVelocityDataObject();
   
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;

   private final ControlFlowOutputPort<PointVelocityDataObject> pointVelocityOutputPort = createOutputPort("pointVelocityOutputPort");

   public SimulatedPointVelocitySensorFromRobot(String name, RigidBody rigidBody,
        ReferenceFrame bodyFrame, KinematicPoint kinematicPoint, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      this.rigidBody = rigidBody;
      
      this.kinematicPoint = kinematicPoint;

      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);
   }

   public void startComputation()
   {
      kinematicPoint.getVelocity(pointVelocity);
      yoFrameVectorPerfect.set(pointVelocity);

      corrupt(pointVelocity);
      yoFrameVectorNoisy.set(pointVelocity);
      
      measurementPointInBodyFrame.setIncludingFrame(bodyFrame, kinematicPoint.getOffsetCopy());
      velocityOfMeasurementPointInWorldFrame.setIncludingFrame(ReferenceFrame.getWorldFrame(), pointVelocity);
      
      boolean isPointVelocityValid = true;
      pointVelocityDataObject.set(rigidBody, measurementPointInBodyFrame, velocityOfMeasurementPointInWorldFrame, isPointVelocityValid);
      pointVelocityOutputPort.setData(pointVelocityDataObject);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<PointVelocityDataObject> getPointVelocityOutputPort()
   {
      return pointVelocityOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}

