package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SimulatedPointVelocitySensorFromRobot extends SimulatedSensor<Tuple3d>
{
   private final KinematicPoint kinematicPoint;
   
   private final RigidBody rigidBody;
   private final ReferenceFrame bodyFrame;
   
   private final Vector3d pointVelocity = new Vector3d();
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

