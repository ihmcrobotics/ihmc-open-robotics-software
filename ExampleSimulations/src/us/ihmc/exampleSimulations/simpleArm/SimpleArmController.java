package us.ihmc.exampleSimulations.simpleArm;


import java.util.Random;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

/**
 * Random controller to test state estimation.
 */
public class SimpleArmController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleRobotInputOutputMap robot;
   private final DoubleYoVariable time;

   private static final Random random = new Random(94929438248L);
   private DoubleYoVariable noiseMagnitude = new DoubleYoVariable("NoiseMagnitude", registry);
   private DoubleYoVariable decay = new DoubleYoVariable("Decay", registry);
   private DoubleYoVariable frequency = new DoubleYoVariable("Frequency", registry);
   private DoubleYoVariable magnitude = new DoubleYoVariable("Magnitude", registry);
   private DoubleYoVariable offset = new DoubleYoVariable("Offset", registry);

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   public SimpleArmController(SimpleRobotInputOutputMap robot, RigidBody endEffectorBody, DoubleYoVariable time)
   {
      this.time = time;
      this.robot = robot;

      noiseMagnitude.set(0.5);
      decay.set(0.05);
      frequency.set(0.5);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(endEffectorBody, SimpleArmRobot.gravity);
   }

   @Override
   public void doControl()
   {
      robot.readFromSimulation();

      magnitude.add(noiseMagnitude.getDoubleValue() * (random.nextDouble() - 0.5));
      offset.add(noiseMagnitude.getDoubleValue() * (random.nextDouble() - 0.5));

      double angle = 2.0 * Math.PI * frequency.getDoubleValue() * time.getDoubleValue();
      double randomTorque = magnitude.getDoubleValue() * Math.sin(angle) + offset.getDoubleValue();

      inverseDynamicsCalculator.compute();

      robot.addYawTorque(randomTorque);
      robot.addPitch1Torque(randomTorque);
      robot.addPitch2Torque(randomTorque);

      magnitude.mul(1.0 - decay.getDoubleValue());
      offset.mul(1.0 - decay.getDoubleValue());

      robot.writeToSimulation();
   }

}
