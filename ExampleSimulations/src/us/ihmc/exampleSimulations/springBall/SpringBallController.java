package us.ihmc.exampleSimulations.springBall;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;

public class SpringBallController implements RobotController
{
   private SpringBallRobot rob;

   private YoDouble t, q_x, q_y, q_z;
   private YoDouble qd_wx, qd_wy, qd_wz;

   private YoDouble[]
      pos = new YoDouble[SpringBallRobot.NUM_SPIKES], vel = new YoDouble[SpringBallRobot.NUM_SPIKES], tau = new YoDouble[SpringBallRobot.NUM_SPIKES];

   private YoVariableRegistry registry = new YoVariableRegistry("SpringBallController");

   private YoDouble offset_spike = new YoDouble("offset_spike", registry), amp_spike = new YoDouble("amp_spike", registry),
                      freq_spike = new YoDouble("freq_spike", registry);
   private YoDouble q_d_spike = new YoDouble("q_d_spike", registry), k_spike = new YoDouble("k_spike", registry),
                      b_spike = new YoDouble("b_spike", registry);


   private YoDouble[] controlVars = new YoDouble[]
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
      t = (YoDouble)rob.getVariable("t");

      q_x = (YoDouble)rob.getVariable("q_x");
      q_y = (YoDouble)rob.getVariable("q_y");
      q_z = (YoDouble)rob.getVariable("q_z");

      qd_wx = (YoDouble)rob.getVariable("qd_wx");
      qd_wy = (YoDouble)rob.getVariable("qd_wy");
      qd_wz = (YoDouble)rob.getVariable("qd_wz");

      q_z.set(1.0);

      for (int i = 0; i < SpringBallRobot.NUM_SPIKES; i++)
      {
         pos[i] = (YoDouble)rob.getVariable("q_slider" + i);
         vel[i] = (YoDouble)rob.getVariable("qd_slider" + i);
         tau[i] = (YoDouble)rob.getVariable("tau_slider" + i);
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


   public YoDouble[] getControlVars()
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
