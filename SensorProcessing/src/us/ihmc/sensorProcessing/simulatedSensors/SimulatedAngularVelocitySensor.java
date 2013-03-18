package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class SimulatedAngularVelocitySensor extends SimulatedSensor<Vector3d>
{
   private final TwistCalculator twistCalculator;
   private final RigidBody rigidBody;
   private final ReferenceFrame measurementFrame;
   private final Twist twist = new Twist();
   private final Vector3d angularVelocity = new Vector3d();
   private final ControlFlowOutputPort<Vector3d> angularVelocityOutputPort = createOutputPort();

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

   public ControlFlowOutputPort<Vector3d> getAngularVelocityOutputPort()
   {
      return angularVelocityOutputPort;
   }
}
