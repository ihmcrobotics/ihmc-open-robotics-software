package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;


public class SimulatedAngularVelocitySensor extends SimulatedSensor<Vector3D>
{
   private final TwistCalculator twistCalculator;
   private final RigidBody rigidBody;
   private final ReferenceFrame measurementFrame;
   private final Twist twist = new Twist();
   
   private final Vector3D angularVelocity = new Vector3D();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;
   
   private final ControlFlowOutputPort<Vector3D> angularVelocityOutputPort = createOutputPort("angularVelocityOutputPort");

   public SimulatedAngularVelocitySensor(String name, TwistCalculator twistCalculator, RigidBody rigidBody, ReferenceFrame measurementFrame, YoVariableRegistry registry)
   {
      this.twistCalculator = twistCalculator;
      this.rigidBody = rigidBody;
      this.measurementFrame = measurementFrame;
      
      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", measurementFrame, registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", measurementFrame, registry);
   }

   public void startComputation()
   {
      twistCalculator.getTwistOfBody(rigidBody, twist);

      twist.changeFrame(measurementFrame);
      twist.getAngularPart(angularVelocity);
      yoFrameVectorPerfect.set(angularVelocity);
      
      corrupt(angularVelocity);
      yoFrameVectorNoisy.set(angularVelocity);

      angularVelocityOutputPort.setData(angularVelocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3D> getAngularVelocityOutputPort()
   {
      return angularVelocityOutputPort;
   }

   public void initialize()
   {
      // empty
   }
}
