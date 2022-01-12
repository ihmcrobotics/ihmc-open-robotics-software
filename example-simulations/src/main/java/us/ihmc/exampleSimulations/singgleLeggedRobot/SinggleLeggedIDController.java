package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.robotics.screwTheory.MassMatrixCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedIDController implements RobotController
{
   // A name for this controller.
   private final String name = "SinggleLeggedIDController";

   // This line instantiates a registry that will contain relevant controller variables 
   // that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry(name);
   private SinggleLeggedRobot robot;

   /* Control variables */

   // Target angle
   private YoDouble desiredHipPosition;
   private YoDouble desiredHipVelocity;
   private YoDouble desiredHipAcceleration;
   private YoDouble desiredHipAngPosition;
   private YoDouble desiredHipAngVelocity;
   private YoDouble desiredHipAngAcceleration;
   private YoDouble desiredKneeAngPosition;
   private YoDouble desiredKneeAngVelocity;
   private YoDouble desiredKneeAngAcceleration;

   // Controller parameter variables
   private double p_gain_hip, p_gain_knee, d_gain_hip, d_gain_knee;

   // Initial Error
   private double positionError_z = 0.0;
   

   private double qErrorHipPitch = 0.0;
   private double qdErrorHipPitch = 0.0;
   

   private double qErrorKneePitch = 0.0;
   private double qdErrorKneePitch = 0.0;
   

   private double setPoint;
   private double qddHipCalculated;
   private double qddKneeCalculated;

   private MatrixForML massMatrix;
   private MatrixForML qddMatrix;
   private MatrixForML externalTorque;
   private MatrixForML activeTorque;
   
   // Other variables
   private double position_z = 0.0;

   /*
    * Constructor: where we instantiate and initialize control variables
    */
   public SinggleLeggedIDController(SinggleLeggedRobot robot)
   {
      this.robot = robot;
      desiredHipPosition = new YoDouble("DesiredHipPos", registry);
      desiredHipVelocity = new YoDouble("DesiredHipVel", registry);
      desiredHipAcceleration = new YoDouble("DesiredHipAcc", registry);
      desiredHipAngPosition = new YoDouble("DesiredHipAngPosition", registry);
      desiredHipAngVelocity = new YoDouble("DesiredHipAngVelocity", registry);
      desiredHipAngAcceleration = new YoDouble("DesiredHipAngAcceleration", registry);
      desiredKneeAngPosition = new YoDouble("DesiredKneeAngPosition", registry);
      desiredKneeAngVelocity = new YoDouble("DesiredKneeAngVelocity", registry);
      desiredKneeAngAcceleration = new YoDouble("DesiredKneeAngAcceleration", registry);
      massMatrix = new MatrixForML(2, 2);
      qddMatrix = new MatrixForML(2, 1);
      externalTorque = new MatrixForML(2, 1);
      activeTorque = new MatrixForML(2, 1);
      initialize();
   }

   private void updateDesiredSetPoint(double hipPosition)
   {
      desiredHipPosition.set(hipPosition);
      desiredHipVelocity.set(0.0);
      desiredHipAcceleration.set(0.0);

      desiredHipAngPosition.set(Math.acos(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H)));
      desiredHipAngVelocity.set(0.0);
      desiredHipAngAcceleration.set(0.0);

      desiredKneeAngPosition.set(-2 * desiredHipAngPosition.getDoubleValue());
      desiredKneeAngVelocity.set(0.0);
      desiredKneeAngAcceleration.set(0.0);
   }

   private void updateTrajectory(YoDouble desiredHipPosition)
   {
      double qHipTrajectory;
      double qdHipTrajectory;
      double qddHipTrajectory;
      double qKneeTrajectory;
      double qdKneeTrajectory;
      double qddKneeTrajectory;

      qHipTrajectory = Math.acos(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H));
      qdHipTrajectory = (qHipTrajectory - desiredHipAngPosition.getDoubleValue()) / SinggleLeggedSimulation.DT;
      qddHipTrajectory = (qdHipTrajectory - desiredHipAngVelocity.getDoubleValue()) / SinggleLeggedSimulation.DT;
      desiredHipAngPosition.set(qHipTrajectory);
      desiredHipAngVelocity.set(qdHipTrajectory);
      desiredHipAngAcceleration.set(qddHipTrajectory);

      qKneeTrajectory = -2 * desiredHipAngPosition.getDoubleValue();
      qdKneeTrajectory = -2 * desiredHipAngVelocity.getDoubleValue();
      qddKneeTrajectory = -2 * desiredHipAngAcceleration.getDoubleValue();
      desiredKneeAngPosition.set(qKneeTrajectory);
      desiredKneeAngVelocity.set(qdKneeTrajectory);
      desiredKneeAngAcceleration.set(qddKneeTrajectory);
   }

   private void sinwaveTrajectory(double amplitude, double frequency, double offset)
   {
      double simTime = robot.getTime();
      double trajectory;

      trajectory = amplitude * Math.sin(2 * Math.PI * frequency * simTime) + offset;
      desiredHipPosition.set(trajectory);
      updateTrajectory(desiredHipPosition);
   }

   private void updateMassMatrix(double th1, double th2)
   {
      double dz_dth1, dz_dth2;
      double l1 = robot.UPPER_LEG_H;
      double l2 = robot.LOWER_LEG_H;
      double hipMass = robot.HIP_MASS;
      double A, B, C, D;

      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1) * Math.cos(th2) - l2 * Math.cos(th1) * Math.sin(th2);
      dz_dth2 = -l2 * Math.cos(th1) * Math.sin(th2) - l2 * Math.sin(th1) * Math.cos(th2);

      A = dz_dth1 * dz_dth1;
      B = dz_dth1 * dz_dth2;
      C = dz_dth1 * dz_dth2;
      D = dz_dth2 * dz_dth2;

      double[][] tempMat = {{A, B}, {C, D}};
      massMatrix.set(tempMat);
      massMatrix = massMatrix.mul(hipMass).div(2.0);
   }
   
   private void updateExternalTorqueMatrix(double th1, double th2)
   {
      double dz_dth1, dz_dth2;
      double l1 = robot.UPPER_LEG_H;
      double l2 = robot.LOWER_LEG_H;
      double hipMass = robot.HIP_MASS;
      double A, B;

      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1) * Math.cos(th2) - l2 * Math.cos(th1) * Math.sin(th2);
      dz_dth2 = -l2 * Math.cos(th1) * Math.sin(th2) - l2 * Math.sin(th1) * Math.cos(th2);

      A = dz_dth1 * robot.getGRFz(); 
      B = dz_dth2 * robot.getGRFz();
      

      double[][] tempMat = {{A}, {B}};
      externalTorque.set(tempMat);
   }

   @Override
   public void doControl()
   {
      if (robot.getTime() > 10.0)
      {
         sinwaveTrajectory(0.1, 0.5, setPoint);
      }
      // Position Error Term in Z.Axis
      position_z = (robot.UPPER_LEG_H * Math.cos(robot.getHipAngularPosition())
            + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition() + robot.getKneeAngularPosition()));
      positionError_z = desiredHipPosition.getDoubleValue() - position_z;

      System.out.println("position_z error : " + positionError_z);
      
      qddHipCalculated = desiredHipAngAcceleration.getDoubleValue() + p_gain_hip * (desiredHipAngPosition.getDoubleValue() - robot.getHipAngularPosition())
            + d_gain_hip * (desiredHipAngVelocity.getDoubleValue() - robot.getHipAngularVelocity());

      qddKneeCalculated = desiredKneeAngAcceleration.getDoubleValue() + p_gain_knee * (desiredKneeAngPosition.getDoubleValue() - robot.getKneeAngularPosition())
            + d_gain_knee * (desiredKneeAngVelocity.getDoubleValue() - robot.getKneeAngularVelocity());

      updateMassMatrix(robot.getHipAngularPosition(), robot.getKneeAngularPosition());
      updateExternalTorqueMatrix(robot.getHipAngularPosition(), robot.getKneeAngularPosition());
      double[][] tempMat = {{qddHipCalculated},{qddKneeCalculated}};
      qddMatrix.set(tempMat);
      
      activeTorque = massMatrix.dot(qddMatrix).add(externalTorque);
      
      robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
      robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));

   }

   @Override
   public void initialize()
   {
      setPoint = 0.4;
      updateDesiredSetPoint(setPoint);

      System.out.println(desiredHipAngPosition.getDoubleValue());
      System.out.println(desiredHipAngVelocity.getDoubleValue());

      p_gain_hip = 200.0;
      p_gain_knee = 2.0 * p_gain_hip;
      d_gain_hip = 50.0;
      d_gain_knee = 2.0 * d_gain_hip;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
