package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimplePendulumController implements Controller
{
   // A name for this controller
   private final String name = "pendulumController";

   // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry("PendulumController");

   // These give the controller read/write access to the fulcrum joint's state, resolved once up front.
   private final OneDoFJointReadOnly fulcrumJoint;
   private final OneDoFJointStateBasics fulcrumOutput;

   /* Control variables: */

   // Target angle
   private YoDouble desiredPositionRadians;

   // Controller parameter variables
   private YoDouble p_gain, d_gain, i_gain;

   // This is the desired torque that we will apply to the fulcrum joint (RevoluteJoint)
   private double torque;

   /* Constructor:
      Where we instantiate and initialize control variables
   */
   public SimplePendulumController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      fulcrumJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("FulcrumPin");
      fulcrumOutput = controllerOutput.getOneDoFJointOutput("FulcrumPin");

      desiredPositionRadians = new YoDouble("DesiredPosRad", registry);
      desiredPositionRadians.set(-1.5);

      p_gain = new YoDouble("ProportionalGain", registry);
      p_gain.set(250.0);
      d_gain = new YoDouble("DerivativeGain", registry);
      d_gain.set(100.0);
      i_gain = new YoDouble("IntegralGain", registry);
      i_gain.set(10.0);
   }

   @Override
   public void initialize()
   {

   }

   private double positionError = 0;
   private double integralError = 0;

   @Override
   public void doControl()
   {

      // ERROR term: Compute the difference between the desired position the pendulum and its current position
      positionError = desiredPositionRadians.getDoubleValue() - fulcrumJoint.getQ();

      // INTEGRAL term: Compute a simple numerical integration of the position error
      integralError += positionError * SimplePendulumSimulation.DT;   //

      // P.I.D
      torque = p_gain.getDoubleValue() * positionError + i_gain.getDoubleValue() * integralError + d_gain.getDoubleValue() * (0 - fulcrumJoint.getQd());

      fulcrumOutput.setEffort(torque);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }
}