package us.ihmc.exampleSimulations.unicycle;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class UnicycleController implements RobotController
{
   private final YoRegistry registry = new YoRegistry("UnicycleController");
   private final String name;

   // used for control
   private final YoDouble back_tau, wheel_tau;

   // robot state
   private final YoDouble body_x;
   private final YoDouble pitch, q_back, q_wheel;
   private final YoDouble pitch_rate, qd_back, qd_wheel;

   // control variables
   private final YoDouble x_d = new YoDouble("x_desired", registry);
   // F_lqr is a state feedback controller computed in Octave after modeling the system
   private static final DMatrixRMaj F_lqr = new DMatrixRMaj(new double[][] {
      {-11.50472, 3.80286, 0.11255, -2.32854, -0.23139, 0.17381},
      {23.17410, 47.00831, 0.42755, 6.15708, 10.47226, 0.85362}});

   private final DMatrixRMaj x = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj u = new DMatrixRMaj(2, 1);
   private final double radius;

   public UnicycleController(UnicycleRobot robot, String name)
   {
      this.name = name;
      this.radius = robot.getWheelRadius();

      back_tau = (YoDouble) robot.findVariable("tau_backJoint");
      wheel_tau = (YoDouble) robot.findVariable("tau_wheelJoint");

      pitch = (YoDouble) robot.findVariable("q_pitch");
      pitch_rate = (YoDouble) robot.findVariable("qd_pitch");

      q_back = (YoDouble) robot.findVariable("q_backJoint");
      qd_back = (YoDouble) robot.findVariable("qd_backJoint");

      q_wheel = (YoDouble) robot.findVariable("q_wheelJoint");
      qd_wheel = (YoDouble) robot.findVariable("qd_wheelJoint");

      body_x = (YoDouble) robot.findVariable("q_x");
   }

   @Override
   public void doControl()
   {
      // the state for the controller model is different from the values read from simulation
      // q1 is -q_back
      // q2 is pitch - q_back
      // q3 is q_wheel

      double q1 = - q_back.getDoubleValue();
      double q2 = pitch.getDoubleValue() - q1;
      double q3 = q_wheel.getDoubleValue();
      double q1d = - qd_back.getDoubleValue();
      double q2d = pitch_rate.getDoubleValue() - q1d;
      double q3d = qd_wheel.getDoubleValue();

      x.set(0, q1);
      x.set(1, q2);
      x.set(2, q3 - x_d.getDoubleValue()/radius);
      x.set(3, q1d);
      x.set(4, q2d);
      x.set(5, q3d);

      // do some control here
      CommonOps_DDRM.mult(F_lqr, x, u);
      back_tau.set(-u.get(0));
      wheel_tau.set(u.get(1));
   }

   @Override
   public void initialize()
   {
      x_d.set(body_x.getDoubleValue());
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

   @Override
   public String getDescription()
   {
      return name;
   }

}
