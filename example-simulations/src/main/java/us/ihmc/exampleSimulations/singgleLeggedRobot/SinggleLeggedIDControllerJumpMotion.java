package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedIDControllerJumpMotion implements RobotController
{
   // A name for this controller.
   private final String name = "SinggleLeggedIDControllerJumpMotion";
   private final YoRegistry registry = new YoRegistry(name);
   private SinggleLeggedRobot robot;
   private SinggleLeggedTrajectoryGenerator trajectoryGenerator;

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
   private YoDouble currentHipPosition;
   private YoDouble currentHipVelocity;

   private int iteration = 0;

   // Controller parameter variables
   private double p_gain_hip, p_gain_knee, d_gain_hip, d_gain_knee;

   // Initial Error
   private double qErrorHipPitch = 0.0;
   private double qdErrorHipPitch = 0.0;
   private double qErrorKneePitch = 0.0;
   private double qdErrorKneePitch = 0.0;
   private double qddHipCalculated;
   private double qddKneeCalculated;
   private double referenceTime;
   private double setPoint;

   private MatrixForML massMatrix;
   private MatrixForML qddMatrix;
   private MatrixForML externalTorque;
   private MatrixForML activeTorque;

   private enum statesForJump
   {
      INITIAL, STANCE, AIR
   };

   private statesForJump FSM;

   public SinggleLeggedIDControllerJumpMotion(SinggleLeggedRobot robot)
   {
      this.robot = robot;
      trajectoryGenerator = new SinggleLeggedTrajectoryGenerator();
      desiredHipPosition = new YoDouble("DesiredHipPos", registry);
      desiredHipVelocity = new YoDouble("DesiredHipVel", registry);
      desiredHipAcceleration = new YoDouble("DesiredHipAcc", registry);
      desiredHipAngPosition = new YoDouble("DesiredHipAngPosition", registry);
      desiredHipAngVelocity = new YoDouble("DesiredHipAngVelocity", registry);
      desiredHipAngAcceleration = new YoDouble("DesiredHipAngAcceleration", registry);
      desiredKneeAngPosition = new YoDouble("DesiredKneeAngPosition", registry);
      desiredKneeAngVelocity = new YoDouble("DesiredKneeAngVelocity", registry);
      desiredKneeAngAcceleration = new YoDouble("DesiredKneeAngAcceleration", registry);

      currentHipPosition = new YoDouble("CurrentHipPosition", registry);
      currentHipVelocity = new YoDouble("CurrentHipVelocity", registry);

      massMatrix = new MatrixForML(2, 2);
      qddMatrix = new MatrixForML(2, 1);
      externalTorque = new MatrixForML(2, 1);
      activeTorque = new MatrixForML(2, 1);
     
      FSM = statesForJump.INITIAL;

      initialize();
   }

   @Override
   public void initialize()
   {
      setPoint = 0.425;
      setPointTrajectory(setPoint);
      currentHipPosition.set(setPoint);
      System.out.println(desiredHipAngPosition.getDoubleValue());
      System.out.println(desiredKneeAngPosition.getDoubleValue());

      p_gain_hip = 250.0;
      p_gain_knee = 2.0 * p_gain_hip;
      d_gain_hip = p_gain_hip * 0.1;
      d_gain_knee = 2.0 * d_gain_hip;

      // just for test

   }

   private void updateDesiredTrajectory()
   {
      switch (FSM)
      {
         case INITIAL:
         {
            System.out.println("Initial state.");
            if (iteration > 2000)
            {
               double stanceTime = 0.5464;
               double[][] alpha = {{0, 129.0308, 0.1718, 0.0063, 218.7268, 0}};
               MatrixForML matAlpha = new MatrixForML(1, 6);
               matAlpha.set(alpha);
               FSM = statesForJump.STANCE;
               referenceTime = robot.getTime();
               trajectoryGenerator.setBazierPolynomialTrajectory(0.0, setPoint, referenceTime, stanceTime, matAlpha);

            }

            break;
         }
         case STANCE:
         {
            System.out.println("Stance state.");
            if (robot.getTime() > referenceTime + 0.510) // Or 
            {
               FSM = statesForJump.AIR;
            }

            else if (robot.getTime() > referenceTime) //
            {
               desiredHipPosition.set(trajectoryGenerator.bazierPolynomialTrajectory(robot.getTime()));
               updateTrajectory(desiredHipPosition);
            }

            break;
         }
         case AIR:
            System.out.println("Jump!.");
            setPointTrajectory(0.30);
            p_gain_hip = 250.0; //200
            p_gain_knee = 2.0 * p_gain_hip;
            d_gain_hip = 100.0; //100
            d_gain_knee = 2.0 * d_gain_hip;
            break;

         default:
            break;
      }
   }

   @Override
   public void doControl()
   {
      System.out.println(iteration);
      iteration++;

      updateCurrentHipPosition();
      updateCurrentHipVelocity();
      updateDesiredTrajectory();
      calculateErrorDynamics();
      calculateDesiredTorques();
      setTorques();
   }

   private void calculateDesiredTorques()
   {
      activeTorque = massMatrix.dot(qddMatrix).add(externalTorque);
   }

   private void updateCurrentHipVelocity()
   {
      currentHipVelocity.set(robot.getHipSlideVelocity());
   }

   private void updateCurrentHipPosition()
   {
      currentHipPosition.set(robot.UPPER_LEG_H * Math.cos(robot.getHipAngularPosition())
            + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition() + robot.getKneeAngularPosition()));
   }

   private void calculateErrorDynamics()
   { 
      qErrorHipPitch = desiredHipAngPosition.getDoubleValue() - robot.getHipAngularPosition();
      qdErrorHipPitch = desiredHipAngVelocity.getDoubleValue() - robot.getHipAngularVelocity();
      qddHipCalculated = desiredHipAngAcceleration.getDoubleValue() + p_gain_hip * qErrorHipPitch + d_gain_hip * qdErrorHipPitch;

      qErrorKneePitch = desiredKneeAngPosition.getDoubleValue() - robot.getKneeAngularPosition();
      qdErrorKneePitch = desiredKneeAngVelocity.getDoubleValue() - robot.getKneeAngularVelocity();
      qddKneeCalculated = desiredKneeAngAcceleration.getDoubleValue() + p_gain_knee * qErrorKneePitch + d_gain_knee * qdErrorKneePitch;

      updateMassMatrix(robot.getHipAngularPosition(), robot.getKneeAngularPosition());
      updateExternalTorqueMatrix(robot.getHipAngularPosition(), robot.getKneeAngularPosition());
      double[][] tempMat = {{qddHipCalculated}, {qddKneeCalculated}};
      qddMatrix.set(tempMat);
   }

   private void updateMassMatrix(double th1, double th2)
   {
      double dz_dth1, dz_dth2;
      double l1 = robot.UPPER_LEG_H;
      double l2 = robot.LOWER_LEG_H;
      double hipMass = robot.HIP_MASS + robot.UPPER_LEG_MASS + robot.LOWER_LEG_MASS; // All mass is lumped at the Hip.
      double A, B, C, D;

      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1 + th2);
      dz_dth2 = -l2 * Math.sin(th1 + th2);

      //            dz_dth1 = -2 * l1 * Math.sin(th1);
      //            dz_dth2 = l1 * Math.sin(th1);

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
      double A, B;

      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1 + th2);
      dz_dth2 = -l2 * Math.sin(th1 + th2);
      
      //            dz_dth1 = -2 * l1 * Math.sin(th1);
      //            dz_dth2 = l1 * Math.sin(th1);

      A = dz_dth1 * robot.getGRFz();
      B = dz_dth2 * robot.getGRFz();

      double[][] tempMat = {{A}, {B}};
      externalTorque.set(tempMat);
   }

   private void setPointTrajectory(double hipPosition)
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
      // z to theta1, theta2
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

   private void setTorques()
   {
      robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
      robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));

      /*
       * If we use Torque Control Mode for jump motion
       */

      //      switch (FSM)
      //      {
      //         case INITIAL:
      //         {
      //            
      //            activeTorque = massMatrix.dot(qddMatrix).add(externalTorque);
      //            robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
      //            robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));
      //            break;
      //         }
      //         case STANCE:
      //         {
      //            if (robot.getTime() > referenceTime + 0.520) // Or 
      //            {
      //
      //            }
      //            
      //            else if(robot.getTime() > referenceTime) //+ 0.621*0.5464
      //            {
      //               robot.setHipTorque(trajectoryGenerator.bazierPolynomialTrajectoryTorque(robot.getTime()).getDoubleValue(0, 0));
      //               robot.setKneeTorque(trajectoryGenerator.bazierPolynomialTrajectoryTorque(robot.getTime()).getDoubleValue(0, 1));
      //            }          
      //            break;
      //         }
      //         case AIR:
      //            activeTorque = massMatrix.dot(qddMatrix).add(externalTorque);
      //            robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
      //            robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));
      //            break;
      //
      //         default:
      //            break;
      //      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
