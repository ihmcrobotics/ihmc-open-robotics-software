package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;


public class SliderController implements RobotController
{
   private static final long serialVersionUID = 607356403836230948L;
   private DoubleYoVariable tau_joint1, tau_joint2, q_joint1, q_joint2, qd_joint1, qd_joint2;
   private DoubleYoVariable k1, k2, k3, k4;

   private YoVariableRegistry registry = new YoVariableRegistry("SliderController");

   public SliderController(SliderRobot rob)
   {
      q_joint1 = (DoubleYoVariable) rob.getVariable("q_joint1");
      qd_joint1 = (DoubleYoVariable) rob.getVariable("qd_joint1");
      tau_joint1 = (DoubleYoVariable) rob.getVariable("tau_joint1");

      q_joint2 = (DoubleYoVariable) rob.getVariable("q_joint2");
      qd_joint2 = (DoubleYoVariable) rob.getVariable("qd_joint2");
      tau_joint2 = (DoubleYoVariable) rob.getVariable("tau_joint2");

      k1 = new DoubleYoVariable("k1", registry);
      k1.set(-70.7107);
      k2 = new DoubleYoVariable("k2", registry);
      k2.set(-27.8345);
      k3 = new DoubleYoVariable("k3", registry);
      k3.set(-25.5298);
      k4 = new DoubleYoVariable("k4", registry);
      k4.set(-20.9238);
   }

   public DoubleYoVariable[] getControlVars()
   {
      return new DoubleYoVariable[] {k1, k2, k3, k4};
   }

   public void doControl()
   {
      tau_joint2.set(0.0);
      tau_joint1.set(k1.getDoubleValue() * (-q_joint2.getDoubleValue()) - k2.getDoubleValue() * qd_joint2.getDoubleValue()
                     - k3.getDoubleValue() * q_joint1.getDoubleValue() - k4.getDoubleValue() * qd_joint1.getDoubleValue());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }


   public String getName()
   {
      return "SliderController";
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
}
