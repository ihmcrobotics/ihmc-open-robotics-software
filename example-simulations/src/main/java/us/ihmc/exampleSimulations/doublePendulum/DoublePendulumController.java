package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePendulumController implements Controller
{

   // joint1/joint2 give read access to position/velocity, joint1Output/joint2Output give write access to torque
   private final OneDoFJointReadOnly joint1, joint2;
   private final OneDoFJointStateBasics joint1Output, joint2Output;
   private YoDouble k1, k2, k3, k4; // these are the controller gain parameters
   private final YoRegistry registry = new YoRegistry("DoublePendulumController");
   private String name;

   public DoublePendulumController(ControllerInput controllerInput, ControllerOutput controllerOutput, String name)
   {
      this.name = name;

      // get joint state handles from the robot
      joint1 = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("joint1");
      joint2 = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("joint2");
      joint1Output = controllerOutput.getOneDoFJointOutput("joint1");
      joint2Output = controllerOutput.getOneDoFJointOutput("joint2");

      // set controller gains
      // gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
      k1 = new YoDouble("k1", registry);
      k1.set(-242.52);
      k2 = new YoDouble("k2", registry);
      k2.set(-96.33);
      k3 = new YoDouble("k3", registry);
      k3.set(-104.59);
      k4 = new YoDouble("k4", registry);
      k4.set(-49.05);
   }

   public void doControl()
   {
      joint1Output.setEffort(0.0); // free bearing
      // set the torque at the controlled second joint
      joint2Output.setEffort(-k1.getDoubleValue() * joint1.getQ() - k2.getDoubleValue() * joint2.getQ() - k3.getDoubleValue() * joint1.getQd()
                              - k4.getDoubleValue() * joint2.getQd());
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public void initialize()
   {
   }
}
