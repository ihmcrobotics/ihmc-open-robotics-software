package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedPIDController implements RobotController
{
   // A name for this controller.
   private final String name = "SinggleLeggedPIDController";

   // This line instantiates a registry that will contain relevant controller variables 
   // that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry(name);
   private SinggleLeggedRobot robot;

   /* Control variables */

   // Target angle
   private YoDouble desiredHipPosition;
   private YoDouble desiredHipVelocity;
   private YoDouble desiredHipAngPosition;
   private YoDouble desiredHipAngVelocity;
   private YoDouble desiredKneeAngPosition;
   private YoDouble desiredKneeAngVelocity;

   // Controller parameter variables
   private double p_gain_hip, p_gain_knee, d_gain_hip, d_gain_knee, i_gain_hip, i_gain_knee;

   // This is the desired torque that we apply to each joints.
   private double torqueHip, torqueKnee;

   // Initial Error
   private double positionError_z = 0.0;
   private double differentialError_z = 0.0;
   private double positionError_hip_theta = 0.0; //TODO : Replace 'theta' to 'pitch'
   private double positionError_knee_theta = 0.0;
   private double differentialError_hip_theta = 0.0;
   private double differentialError_knee_theta = 0.0;
   private double integralError_hip_theta = 0.0;
   private double integralError_knee_theta = 0.0;

   // First Set point
   private double setPoint;

   // Other variables
   private double position_z = 0.0;

   /*
    * Constructor: where we instantiate and initialize control variables
    */
   public SinggleLeggedPIDController(SinggleLeggedRobot robot)
   {
      this.robot = robot;
      desiredHipPosition = new YoDouble("DesiredHipPos", registry);
      desiredHipVelocity = new YoDouble("DesiredHipVel", registry);
      desiredHipAngPosition = new YoDouble("DesiredHipAngPosition", registry);
      desiredHipAngVelocity = new YoDouble("DesiredHipAngVelocity", registry);
      desiredKneeAngPosition = new YoDouble("DesiredKneeAngPosition", registry);
      desiredKneeAngVelocity = new YoDouble("DesiredKneeAngVelocity", registry);

      initialize();

   }

   private void updateDesiredSetPoint(double hipPosition)
   {
      desiredHipPosition.set(hipPosition);
      desiredHipVelocity.set(0.0);

      desiredHipAngPosition.set(Math.acos(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H)));
      desiredHipAngVelocity.set(0.0);

      desiredKneeAngPosition.set(-2 * desiredHipAngPosition.getDoubleValue());
      desiredKneeAngVelocity.set(0.0);
   }

   private void updateTrajectory(YoDouble desiredHipPosition, YoDouble desiredHipVelocity)
   {
      double qHipTrajectory;
      double qdHipTrajectory;
      double qKneeTrajectory;
      double qdKneeTrajectory;

      qHipTrajectory = Math.acos(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H));
      qdHipTrajectory = (qHipTrajectory - desiredHipAngPosition.getDoubleValue()) / SinggleLeggedSimulation.DT;
      desiredHipAngPosition.set(qHipTrajectory);
      desiredHipAngVelocity.set(qdHipTrajectory);
      // Closed form
      //      desiredHipAngVelocity.set(desiredHipVelocity.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H) * -1
      //            / Math.sqrt(1 - Math.pow(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H), 2.0)));

      qKneeTrajectory = -2 * desiredHipAngPosition.getDoubleValue();
      qdKneeTrajectory = -2 * desiredHipAngVelocity.getDoubleValue();
      desiredKneeAngPosition.set(qKneeTrajectory);
      desiredKneeAngVelocity.set(qdKneeTrajectory);
   }

   private void sinwaveTrajectory(double Amplitude, double frequency)
   {
      double simTime = robot.getTime();
      double trajectory;
      double dtrajectory;

      trajectory = Amplitude * Math.sin(2 * Math.PI * frequency * simTime) + 0.4;
      dtrajectory = (trajectory - desiredHipPosition.getDoubleValue()) / SinggleLeggedSimulation.DT;
      desiredHipPosition.set(trajectory);
      desiredHipVelocity.set(dtrajectory);
      updateTrajectory(desiredHipPosition, desiredHipVelocity);
   }

   @Override
   public void doControl()
   {
      if (robot.getTime() > 10.0)
      {
         sinwaveTrajectory(0.1, 0.2);
      }
      
      /*
       * Just for calculate the errors in Hip_z
       */
      // Position Error Term in Z.Axis
      position_z = (robot.UPPER_LEG_H * Math.cos(robot.getHipAngularPosition())
            + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition() + robot.getKneeAngularPosition()));
      positionError_z = desiredHipPosition.getDoubleValue() - position_z;

      System.out.println("position_z error : " + positionError_z);

      // Differential Error Term in Z.Axis
      differentialError_z = desiredHipVelocity.getDoubleValue() - (-robot.getHipAngularVelocity()
            * (robot.UPPER_LEG_H * Math.sin(robot.getHipAngularPosition())
                  + robot.LOWER_LEG_H * Math.sin(robot.getHipAngularPosition()) * Math.cos(robot.getKneeAngularPosition())
                  + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition()) * Math.sin(robot.getKneeAngularPosition()))
            - robot.getKneeAngularVelocity() * robot.LOWER_LEG_H * (Math.cos(robot.getHipAngularPosition()) * Math.sin(robot.getKneeAngularPosition())
                  + Math.sin(robot.getHipAngularPosition()) * Math.cos(robot.getKneeAngularPosition())));
      //      System.out.println("differential_z error : " + differentialError_z);

      /*
       * Calculate the Errors for Control
       */
      // Position Error Term in Pitch.Axis      
      positionError_hip_theta = desiredHipAngPosition.getDoubleValue() - robot.getHipAngularPosition();
      positionError_knee_theta = desiredKneeAngPosition.getDoubleValue() - robot.getKneeAngularPosition();

      // Differential Error Term in Pitch.Axis      
      differentialError_hip_theta = desiredHipAngVelocity.getDoubleValue() - robot.getHipAngularVelocity();
      differentialError_knee_theta = desiredKneeAngVelocity.getDoubleValue() - robot.getKneeAngularVelocity();

      // Integral Error term in Pitch.Axis
      integralError_hip_theta += positionError_hip_theta * SinggleLeggedSimulation.DT;
      integralError_knee_theta += positionError_knee_theta * SinggleLeggedSimulation.DT;

      //      System.out.println("Hip_pos_err : " + positionError_hip_theta);
      //      System.out.println("Hip_diff_err : " + differentialError_hip_theta);
      //      System.out.println("Knee_pos_err : " + positionError_knee_theta);
      //      System.out.println("Knee_diff_err : " + differentialError_knee_theta);

      torqueHip = p_gain_hip * positionError_hip_theta + d_gain_hip * differentialError_hip_theta + i_gain_hip * integralError_hip_theta;
      torqueKnee = p_gain_knee * positionError_knee_theta + d_gain_knee * differentialError_knee_theta + i_gain_knee * integralError_knee_theta;

      // Torque set
      robot.setHipTorque(torqueHip);
      robot.setKneeTorque(torqueKnee);

      // Angular velocity set
      // robot.setHipAngVelocity(torqueHip * SinggleLeggedSimulation.DT);
      // robot.setKneeAngVelocity(torqueKnee * SinggleLeggedSimulation.DT);
   }

   @Override
   public void initialize()
   {
      setPoint = 0.4;
      updateDesiredSetPoint(setPoint);

      System.out.println(desiredHipAngPosition.getDoubleValue());
      System.out.println(desiredHipAngVelocity.getDoubleValue());

      p_gain_hip = 100.0;
      p_gain_knee = 2.0 * p_gain_hip;
      d_gain_hip = 0.5;
      d_gain_knee = 2.0 * d_gain_hip;
      i_gain_hip = 0.0;
      i_gain_knee = 2.0 * i_gain_hip;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
