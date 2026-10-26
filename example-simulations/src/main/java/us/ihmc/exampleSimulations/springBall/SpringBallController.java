package us.ihmc.exampleSimulations.springBall;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SpringBallController implements Controller
{
   private final ControllerInput controllerInput;

   private final OneDoFJointReadOnly[] slider = new OneDoFJointReadOnly[SpringBallRobotDefinition.NUM_SPIKES];
   private final OneDoFJointStateBasics[] tau = new OneDoFJointStateBasics[SpringBallRobotDefinition.NUM_SPIKES];

   private YoRegistry registry = new YoRegistry("SpringBallController");

   private YoDouble offset_spike = new YoDouble("offset_spike", registry), amp_spike = new YoDouble("amp_spike", registry),
                      freq_spike = new YoDouble("freq_spike", registry);
   private YoDouble q_d_spike = new YoDouble("q_d_spike", registry), k_spike = new YoDouble("k_spike", registry),
                      b_spike = new YoDouble("b_spike", registry);


   private YoDouble[] controlVars = new YoDouble[]
   {
      offset_spike, amp_spike, freq_spike, q_d_spike, k_spike, b_spike
   };

   private String name;

   public SpringBallController(ControllerInput controllerInput, ControllerOutput controllerOutput, String name)
   {
      this.controllerInput = controllerInput;
      this.name = name;
      initControl(controllerInput, controllerOutput);
   }

   private void initControl(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      for (int i = 0; i < SpringBallRobotDefinition.NUM_SPIKES; i++)
      {
         slider[i] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("slider" + i);
         tau[i] = controllerOutput.getOneDoFJointOutput("slider" + i);
      }

      q_d_spike.set(0.0);
      k_spike.set(100.0);
      b_spike.set(2.0);

      offset_spike.set(-0.1);
      amp_spike.set(0.1);
      freq_spike.set(1.0);
   }


   public YoDouble[] getControlVars()
   {
      return controlVars;
   }

   public void doControl()
   {
      q_d_spike.set(offset_spike.getDoubleValue()
                     + amp_spike.getDoubleValue() * Math.cos(2.0 * Math.PI * freq_spike.getDoubleValue() * controllerInput.getTime()));

      for (int i = 0; i < SpringBallRobotDefinition.NUM_SPIKES; i++)
      {
         tau[i].setEffort(k_spike.getDoubleValue() * (q_d_spike.getDoubleValue() - slider[i].getQ()) - b_spike.getDoubleValue() * slider[i].getQd());
      }

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
