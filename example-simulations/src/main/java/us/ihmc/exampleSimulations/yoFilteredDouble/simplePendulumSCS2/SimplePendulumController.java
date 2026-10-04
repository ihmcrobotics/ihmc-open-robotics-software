package us.ihmc.exampleSimulations.yoFilteredDouble.simplePendulumSCS2;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.math.filters.ContinuousTransferFunction;
import us.ihmc.robotics.math.filters.TransferFunctionDiscretizer;
import us.ihmc.robotics.math.filters.YoFilteredDouble;
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

   // This line instantiates a registry that will contain relevant controller
   // variables that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry("PIDControl");

   // Instance of our joint
   private OneDoFJointReadOnly fulcrumJoint;
   private OneDoFJointStateBasics fulcrumJointCommand;

   /* Control variables: */

   // Target angle
   private YoDouble desiredPositionRadians;

   // This is the desired torque that we will apply to the fulcrum joint (PinJoint)
   private double torque;

   // PID Parameters
   private final double Kp = 250.0;
   private final double Ki = 10.0;
   private final double Kd = 100.0;
   private final double Tau = 0.35;

   // Notch Filter Parameters
   private final double wn = 60 * 2 * Math.PI;
   private final double Q = 1.0;

   // Controller Frequency
   private YoFilteredDouble Controller_Var;            // PID Controller Double

   // Constructor: Where we instantiate and initialize control variables
   public SimplePendulumController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      desiredPositionRadians = new YoDouble("DesiredPosRad", registry);
      desiredPositionRadians.set(-1.5); // set initial position of the pendulum

      // Get the objects of the fulcrum joint to read input and write output in the control loop
      this.fulcrumJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SimplePendulumDefinition.jointName);
      this.fulcrumJointCommand = controllerOutput.getOneDoFJointOutput(fulcrumJoint);

      // Set up PID Transfer Function and use TransferFunctionDiscretizer to solve for the discrete TF coefficients.
      ContinuousTransferFunction PID_TF = new ContinuousTransferFunction("PID",
                                                                         1.0,
                                                                         new double[] {(Kp + Tau * Kd), (Tau * Kp + Ki), Ki * Tau},
                                                                         new double[] {1.0, Tau, 0.0});

      // Discretize the PID TF into a filter for a YoFilteredDouble
      TransferFunctionDiscretizer PID_Filter = new TransferFunctionDiscretizer(PID_TF, 1 / SimplePendulumSimulation.DT);

      // Pass the Filter into the a YoFilteredDouble object with safe startup active.
      Controller_Var = new YoFilteredDouble("Controller_Var", registry, PID_Filter, true);
   }

   @Override
   public void initialize()
   {
   }

   private double positionError = 0;

   @Override
   public void doControl()
   {
      // ERROR term: Compute the difference between the desired position the pendulum
      // and its current position	  
      positionError = desiredPositionRadians.getDoubleValue() - fulcrumJoint.getQ();

      // Pass the error term into the input of the filter.
      Controller_Var.set(positionError);

      // Retrieve the filter output as the new torque command.
      torque = Controller_Var.getFilteredValue();

      // Set the desired torque for the fulcrum joint as controller output
      this.fulcrumJointCommand.setEffort(torque);
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