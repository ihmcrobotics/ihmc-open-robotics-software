package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;


public class SimulatedPointVelocitySensor extends SimulatedSensor<Vector3D>
{
   private final RigidBody rigidBody;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint3D pointToMeasureVelocityOf;
   private final FramePoint3D tempPointToMeasureVelocityOf = new FramePoint3D();
   private final FrameVector3D pointVelocityFrameVector = new FrameVector3D();

   private final Twist twist = new Twist();

   private final Vector3D pointVelocity = new Vector3D();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;

   private final TwistCalculator twistCalculator;

   private final ControlFlowOutputPort<Vector3D> pointVelocityOutputPort = createOutputPort("pointVelocityOutputPort");

   public SimulatedPointVelocitySensor(String name, RigidBody rigidBody, FramePoint3D pointToMeasureVelocityOf,
           TwistCalculator twistCalculator, YoVariableRegistry registry)
   {
      this.rigidBody = rigidBody;
      this.twistCalculator = twistCalculator;

      this.pointToMeasureVelocityOf = new FramePoint3D(pointToMeasureVelocityOf);

      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);
   }

   public void startComputation()
   {
      twistCalculator.getTwistOfBody(rigidBody, twist);
      twist.changeFrame(twist.getBaseFrame());
      
      tempPointToMeasureVelocityOf.setIncludingFrame(pointToMeasureVelocityOf);
      tempPointToMeasureVelocityOf.changeFrame(twist.getBaseFrame());
      twist.getLinearVelocityOfPointFixedInBodyFrame(pointVelocityFrameVector, tempPointToMeasureVelocityOf);
      
      pointVelocityFrameVector.changeFrame(worldFrame);
      pointVelocityFrameVector.get(pointVelocity);
      
      yoFrameVectorPerfect.set(pointVelocity);

      corrupt(pointVelocity);
      yoFrameVectorNoisy.set(pointVelocity);
      
      pointVelocityOutputPort.setData(pointVelocity);
   }
   
   public void packPointVelocity(FrameVector3D velocity)
   {
      yoFrameVectorPerfect.getFrameTupleIncludingFrame(velocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3D> getPointVelocityOutputPort()
   {
      return pointVelocityOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
