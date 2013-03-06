package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptorHolder;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class SimulatedAngularVelocitySensor extends SignalCorruptorHolder<FrameVector> implements ControlFlowElement
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TwistCalculator twistCalculator;
   private final RigidBody rigidBody;
   private final ReferenceFrame measurementFrame;
   private final Twist twist = new Twist();
   private final FrameVector angularVelocity = new FrameVector(worldFrame);
   private final ControlFlowOutputPort<FrameVector> angularVelocityOutputPort = new ControlFlowOutputPort<FrameVector>(this);

   public SimulatedAngularVelocitySensor(TwistCalculator twistCalculator, RigidBody rigidBody, ReferenceFrame measurementFrame)
   {
      this.twistCalculator = twistCalculator;
      this.rigidBody = rigidBody;
      this.measurementFrame = measurementFrame;
   }

   public void startComputation()
   {
      twistCalculator.packTwistOfBody(twist, rigidBody);

      twist.changeFrame(measurementFrame);
      twist.packAngularPart(angularVelocity);
      corrupt(angularVelocity);

      angularVelocityOutputPort.setData(angularVelocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<FrameVector> getAngularVelocityOutputPort()
   {
      return angularVelocityOutputPort;
   }
}
