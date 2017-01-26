package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

/**
 * Created by amoucheboeuf on 2/11/16.
 */
public class SimplePendulumController implements RobotController
{
   // A name for this controller
   private final String name = "pendulumController";

   // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
   private final YoVariableRegistry registry = new YoVariableRegistry("PendulumController");

   // This is a reference to the SimplePendulumRobot that enables the controller to access this robot's variables.
   private SimplePendulumRobot robot;

   /* Control variables: */

   // Target angle
   private DoubleYoVariable desiredPositionRadians;

   // Controller parameter variables
   private DoubleYoVariable p_gain, d_gain, i_gain;

   // This is the desired torque that we will apply to the fulcrum joint (PinJoint)
   private double torque;

   /* Constructor:
      Where we instantiate and initialize control variables
   */
   public SimplePendulumController(SimplePendulumRobot robot)
   {
      this.robot = robot;
      desiredPositionRadians = new DoubleYoVariable("DesiredPosRad", registry);
      desiredPositionRadians.set(-1.5);

      p_gain = new DoubleYoVariable("ProportionalGain", registry);
      p_gain.set(250.0);
      d_gain = new DoubleYoVariable("DerivativeGain", registry);
      d_gain.set(100.0);
      i_gain = new DoubleYoVariable("IntegralGain", registry);
      i_gain.set(10.0);
   }

   @Override public void initialize()
   {

   }

   private double positionError = 0;
   private double integralError = 0;

   @Override public void doControl()
   {

      // ERROR term: Compute the difference between the desired position the pendulum and its current position
      positionError = desiredPositionRadians.getDoubleValue() - robot.getFulcrumAngularPosition();

      // INTEGRAL term: Compute a simple numerical integration of the position error
      integralError += positionError * SimplePendulumSimulation.DT;   //

      // P.I.D
      torque = p_gain.getDoubleValue() * positionError + i_gain.getDoubleValue() * integralError + d_gain.getDoubleValue() * (0 - robot
            .getFulcrumAngularVelocity());

      robot.setFulcrumTorque(torque);

   }

   @Override public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override public String getName()
   {
      return name;
   }

   @Override public String getDescription()
   {
      return name;
   }
}

//lastPositionError = positionError;
//   private double lastPositionError = 0;
//      integralError += (positionError + lastPositionError)/2.0 * SimplePendulumSimulation.DT;   // INTEGRAL