package us.ihmc.exampleSimulations.simplePendulum2;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * 
 * @author amoucheboeuf 
 * mod by jaehoon
 *
 */
public class SimplePendulumController2 implements RobotController
{
   // A name for this controller
   private final String name = "pendulumController";

   // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry("pendulumController");

   // This is a reference to the SimplePendulumRobot that enables the controller to access this robot's variables.
   private SimplePendulumRobot2 robot;

   /* Control variables */

   // Target angle
   private YoDouble desiredPositionRadians;
   private YoDouble desiredVelocityRadians;

   // Controller parameter variables
   private YoDouble p_gain, d_gain, i_gain;

   // This is the desired torque that we will apply to the fulcrum joint (Pin Joint)
   private double torque;

   // Initial Error
   private double positionError = 0;
   private double integralError = 0;
   private double differentialError = 0;
   
   /*
    * Constructor: where we instantiate and initialize control variables
    */
   public SimplePendulumController2(SimplePendulumRobot2 robot)
   {
      this.robot = robot;
      desiredPositionRadians = new YoDouble("DesiredPosRad", registry);
      desiredPositionRadians.set(0.0);
      desiredVelocityRadians = new YoDouble("DesiredVelRad", registry);
      desiredVelocityRadians.set(0.5);

      p_gain = new YoDouble("ProportionalGain", registry);
      p_gain.set(0.0);
      d_gain = new YoDouble("DerivativeGain", registry);
      d_gain.set(0.0);
      i_gain = new YoDouble("IntegralGain", registry);
      i_gain.set(0.0);
   }
   
   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
      // Position Error Term
      positionError = desiredPositionRadians.getDoubleValue() - robot.getFulcrumAngularPosition();
      
      // Integral Error Term
      integralError += positionError * SimplePendulumSimulation2.DT;
      
      // Differential Error Term
      differentialError = desiredVelocityRadians.getDoubleValue() - robot.getFulcrumAngularVelocity();
      
      // P.I.D. Controller
      torque = p_gain.getDoubleValue() * positionError + i_gain.getDoubleValue() * integralError + d_gain.getDoubleValue() * differentialError ;
      
      // torque set
      robot.setFulcrumTorque(torque);
      
   }
   
   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   

}
