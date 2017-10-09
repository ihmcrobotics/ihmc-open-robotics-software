package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;


public class SimulatedLinearAccelerationSensor extends SimulatedSensor<Vector3D>
{
   private final RigidBody rigidBody;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint3D imuFramePoint = new FramePoint3D(worldFrame);

   private final FrameVector3D linearAccelerationFrameVector = new FrameVector3D(worldFrame);
   private final Vector3D linearAcceleration = new Vector3D();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;

   private final ReferenceFrame measurementFrame;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final ControlFlowOutputPort<Vector3D> linearAccelerationOutputPort = createOutputPort("linearAccelerationOutputPort");
   private final FrameVector3D gravitationalAcceleration;

   public SimulatedLinearAccelerationSensor(String name, RigidBody rigidBody, ReferenceFrame measurementFrame,
           SpatialAccelerationCalculator spatialAccelerationCalculator, Vector3D gravitationalAcceleration, YoVariableRegistry registry)
   {
      this.rigidBody = rigidBody;
      this.measurementFrame = measurementFrame;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
      this.gravitationalAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), gravitationalAcceleration);
      
      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", measurementFrame, registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", measurementFrame, registry);
   }

   public void startComputation()
   {
      imuFramePoint.setToZero(measurementFrame);
      RigidBody rootBody = spatialAccelerationCalculator.getRootBody();
      spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(rootBody, rigidBody, imuFramePoint, linearAccelerationFrameVector);
      linearAccelerationFrameVector.changeFrame(gravitationalAcceleration.getReferenceFrame());
      linearAccelerationFrameVector.add(gravitationalAcceleration);
      linearAccelerationFrameVector.changeFrame(measurementFrame);
      linearAccelerationFrameVector.get(linearAcceleration);
      yoFrameVectorPerfect.set(linearAcceleration);

      corrupt(linearAcceleration);
      yoFrameVectorNoisy.set(linearAcceleration);

      linearAccelerationOutputPort.setData(linearAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3D> getLinearAccelerationOutputPort()
   {
      return linearAccelerationOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
