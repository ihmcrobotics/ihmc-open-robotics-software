package us.ihmc.exampleSimulations.springBall;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class SpringBallController implements RobotController
{
   private SpringBallRobot rob;

   private DoubleYoVariable t, q_x, q_y, q_z;
   private DoubleYoVariable qd_wx, qd_wy, qd_wz;

   private DoubleYoVariable[]
      pos = new DoubleYoVariable[SpringBallRobot.NUM_SPIKES], vel = new DoubleYoVariable[SpringBallRobot.NUM_SPIKES], tau = new DoubleYoVariable[SpringBallRobot.NUM_SPIKES];

   private YoVariableRegistry registry = new YoVariableRegistry("SpringBallController");

   private DoubleYoVariable offset_spike = new DoubleYoVariable("offset_spike", registry), amp_spike = new DoubleYoVariable("amp_spike", registry),
                      freq_spike = new DoubleYoVariable("freq_spike", registry);
   private DoubleYoVariable q_d_spike = new DoubleYoVariable("q_d_spike", registry), k_spike = new DoubleYoVariable("k_spike", registry),
                      b_spike = new DoubleYoVariable("b_spike", registry);


   private DoubleYoVariable[] controlVars = new DoubleYoVariable[]
   {
      offset_spike, amp_spike, freq_spike, q_d_spike, k_spike, b_spike
   };

   private String name;

   public SpringBallController(SpringBallRobot rob, String name)
   {
      this.name = name;
      this.rob = rob;
      initControl();
   }

   private void initControl()
   {
      t = (DoubleYoVariable)rob.getVariable("t");

      q_x = (DoubleYoVariable)rob.getVariable("q_x");
      q_y = (DoubleYoVariable)rob.getVariable("q_y");
      q_z = (DoubleYoVariable)rob.getVariable("q_z");

      qd_wx = (DoubleYoVariable)rob.getVariable("qd_wx");
      qd_wy = (DoubleYoVariable)rob.getVariable("qd_wy");
      qd_wz = (DoubleYoVariable)rob.getVariable("qd_wz");

      q_z.set(1.0);

      for (int i = 0; i < SpringBallRobot.NUM_SPIKES; i++)
      {
         pos[i] = (DoubleYoVariable)rob.getVariable("q_slider" + i);
         vel[i] = (DoubleYoVariable)rob.getVariable("qd_slider" + i);
         tau[i] = (DoubleYoVariable)rob.getVariable("tau_slider" + i);
      }

      q_d_spike.set(0.0);
      k_spike.set(100.0);
      b_spike.set(2.0);

      offset_spike.set(-0.1);
      amp_spike.set(0.1);
      freq_spike.set(1.0);

      qd_wz.set(2.0);
      qd_wy.set(1.0);
      qd_wx.set(-1.5);
   }


   public DoubleYoVariable[] getControlVars()
   {
      return controlVars;
   }

   public void doControl()
   {
      q_d_spike.set(offset_spike.getDoubleValue() + amp_spike.getDoubleValue() * Math.cos(2.0 * Math.PI * freq_spike.getDoubleValue() * t.getDoubleValue()));

      for (int i = 0; i < SpringBallRobot.NUM_SPIKES; i++)
      {
         tau[i].set(k_spike.getDoubleValue() * (q_d_spike.getDoubleValue() - pos[i].getDoubleValue()) - b_spike.getDoubleValue() * vel[i].getDoubleValue());
      }

   }

   public YoVariableRegistry getYoVariableRegistry()
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

   public String getDescription()
   {
      return getName();
   }
}
