package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedIDController implements RobotController
{
   // A name for this controller.
   private final String name = "SinggleLeggedIDController";
   private final YoRegistry registry = new YoRegistry(name);
   private SinggleLeggedRobot robot;
   private SinggleLeggedTrajectoryGenerator trajectoryGenerator;

   // Target angle
   private YoDouble desiredHipPosition;
   private YoDouble desiredHipVelocity;
   private YoDouble desiredHipAcceleration;
   private YoDouble desiredHipAngularPosition;
   private YoDouble desiredHipAngularVelocity;
   private YoDouble desiredHipAngularAcceleration;
   private YoDouble desiredKneeAngularPosition;
   private YoDouble desiredKneeAngularVelocity;
   private YoDouble desiredKneeAngularAcceleration;
   private YoDouble currentHipPosition;
   private YoDouble currentHipVelocity;
   private double previousHipPosition;
   private double previousDesiredHipPosition = 0.0;

   // Controller parameter variables
   private double p_gain_hip, p_gain_knee, d_gain_hip, d_gain_knee;

   // These variables will be used for ID controller.
   private double qddHipCalculated;
   private double qddKneeCalculated;

   // Initial Error
   private double qErrorHipPitch = 0.0;
   private double qdErrorHipPitch = 0.0;
   private double qErrorKneePitch = 0.0;
   private double qdErrorKneePitch = 0.0;

   // Matrix
   private MatrixForML massMatrix;
   private MatrixForML qddMatrix;
   private MatrixForML externalTorque;
   private MatrixForML activeTorque;

   private enum trajectory
   {
      SETPOINT, SINWAVE, POLYNOMIAL, BAZIER
   };

   private trajectory trajectoryType;

   private boolean dataCollect;

   // For set point trajectory
   private double setPoint;

   // For polynomial trajectory
   private boolean initPolynomialTrajectory;
   private double td = 0.1; // difference between current time and desired time

   // Other variables
   private double position_z = 0.0; // calculated current hip z position
   private int iteration = 0;

   // For GRFLearning
   private GRFLearningDataCollector grfLearning;
   private int numOfDataPerTrajectory;

   // For Consider Nyquist theorem
   private int setTauValidity;

   private double referenceTime;

   /*
    * Constructor: where we instantiate and initialize control variables
    */
   public SinggleLeggedIDController(SinggleLeggedRobot robot)
   {
      this.robot = robot;
      trajectoryGenerator = new SinggleLeggedTrajectoryGenerator();
      
      desiredHipPosition = new YoDouble("DesiredHipPos", registry);
      desiredHipVelocity = new YoDouble("DesiredHipVel", registry);
      desiredHipAcceleration = new YoDouble("DesiredHipAcc", registry);
      desiredHipAngularPosition = new YoDouble("DesiredHipAngularPosition", registry);
      desiredHipAngularVelocity = new YoDouble("DesiredHipAngularVelocity", registry);
      desiredHipAngularAcceleration = new YoDouble("DesiredHipAngularAcceleration", registry);
      desiredKneeAngularPosition = new YoDouble("DesiredKneeAngularPosition", registry);
      desiredKneeAngularVelocity = new YoDouble("DesiredKneeAngularVelocity", registry);
      desiredKneeAngularAcceleration = new YoDouble("DesiredKneeAngularAcceleration", registry);
      currentHipPosition = new YoDouble("CurrentHipPosition", registry);
      currentHipVelocity = new YoDouble("CurrentHipVelocity", registry);
      
      massMatrix = new MatrixForML(2, 2);
      qddMatrix = new MatrixForML(2, 1);
      externalTorque = new MatrixForML(2, 1);
      activeTorque = new MatrixForML(2, 1);
      
      trajectoryType = trajectory.SETPOINT;
      dataCollect = false;

      initialize();
   }

   @Override
   public void initialize()
   {
      switch (trajectoryType)
      {
         case SETPOINT:
            break;
         case SINWAVE:
            numOfDataPerTrajectory = (int) (10 / (SinggleLeggedSimulation.DT));
            break;
         case POLYNOMIAL:
            numOfDataPerTrajectory = (int) (td / (SinggleLeggedSimulation.DT));
            break;
         default:
            System.out.println("Unavailable trajectory for collecting data.");
            break;
      }
      grfLearning = new GRFLearningDataCollector(robot, numOfDataPerTrajectory);
      
      setPoint = 0.125;
      setPointTrajectory(setPoint);
      currentHipPosition.set(setPoint);

      p_gain_hip = 300.0;
      p_gain_knee = 2.0 * p_gain_hip;
      d_gain_hip = p_gain_hip * 0.1;
      d_gain_knee = 2.0 * d_gain_hip;

      setTauValidity = 0;

      // just for test
      trajectoryGenerator.nextSinwaveTrajectory(0.425, 0.10, 1.0);
   }

   @Override
   public void doControl()
   {
      System.out.println(iteration);
      iteration++;

      generateTrajectory();
      updateCurrentHipPosition();
      updateCurrentHipVelocity();
      calculateErrorDynamics();
      calculateDesiredTorques();
      setTorques();
   }

   private void generateTrajectory()
   {
      if (dataCollect)
      {
         TrajectoryForCollectingData();
      }
      else
      {
         switch (trajectoryType)
         {
            case SETPOINT:
               setPointTrajectory(setPoint);
               break;
            case SINWAVE:
               sinWaveTrajectory();
               break;
            case POLYNOMIAL:

               break;
            case BAZIER:

               break;
         }
      }
   }

   private void sinWaveTrajectory()
   {
      previousHipPosition = desiredHipPosition.getDoubleValue();
      desiredHipPosition.set(trajectoryGenerator.sinwaveTrajectory());
      desiredHipVelocity.set((desiredHipPosition.getDoubleValue() - previousHipPosition) / SinggleLeggedSimulation.DT);
      updateTrajectory(desiredHipPosition);
   }

   private void updateCurrentHipPosition()
   {
      currentHipPosition.set(robot.UPPER_LEG_H * Math.cos(robot.getHipAngularPosition())
            + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition() + robot.getKneeAngularPosition()));
   }

   private void updateCurrentHipVelocity()
   {
      currentHipVelocity.set(robot.getHipSlideVelocity());
   }

   private void calculateErrorDynamics()
   {
      // Calculate for error dynamics
      qErrorHipPitch = desiredHipAngularPosition.getDoubleValue() - robot.getHipAngularPosition();
      qdErrorHipPitch = desiredHipAngularVelocity.getDoubleValue() - robot.getHipAngularVelocity();
      qddHipCalculated = desiredHipAngularAcceleration.getDoubleValue() + p_gain_hip * qErrorHipPitch + d_gain_hip * qdErrorHipPitch;

      qErrorKneePitch = desiredKneeAngularPosition.getDoubleValue() - robot.getKneeAngularPosition();
      qdErrorKneePitch = desiredKneeAngularVelocity.getDoubleValue() - robot.getKneeAngularVelocity();
      qddKneeCalculated = desiredKneeAngularAcceleration.getDoubleValue() + p_gain_knee * qErrorKneePitch + d_gain_knee * qdErrorKneePitch;

      // Error dynamics
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
      double dz_dth1, dz_dth2, dx_dth1, dx_dth2;
      double l1 = robot.UPPER_LEG_H;
      double l2 = robot.LOWER_LEG_H;
      double A, B;

      

      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1 + th2);
      dz_dth2 = -l2 * Math.sin(th1 + th2);

      A = dz_dth1 * robot.getGRFz();
      B = dz_dth2 * robot.getGRFz();

      //      Consider GRFx model (Maybe used in future) 
      //      dx_dth1 = -l1 * Math.cos(th1) - l2 * Math.cos(th1 + th2);
      //      dx_dth2 = -l2 * Math.cos(th1 + th2);
      //      A = dx_dth1 * robot.getGRFx() + dz_dth1 * robot.getGRFz();
      //      B = dx_dth2 * robot.getGRFx() + dz_dth2 * robot.getGRFz();

      double[][] tempMat = {{A}, {B}};
      externalTorque.set(tempMat);
   }

   private void calculateDesiredTorques()
   {
      activeTorque = massMatrix.dot(qddMatrix).add(externalTorque);
   }

   private void setTorques()
   {
      if (dataCollect)
      {
         setTauValidity++;
         if (setTauValidity % 5 == 0)
         {
            robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
            robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));
         }
      }
      else
      {
         robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
         robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));
      }
   }

   private void setPointTrajectory(double hipPosition)
   {
      desiredHipPosition.set(hipPosition);
      desiredHipVelocity.set(0.0);
      desiredHipAcceleration.set(0.0);

      desiredHipAngularPosition.set(Math.acos(desiredHipPosition.getDoubleValue() / (robot.UPPER_LEG_H + robot.LOWER_LEG_H)));
      desiredHipAngularVelocity.set(0.0);
      desiredHipAngularAcceleration.set(0.0);

      desiredKneeAngularPosition.set(-2 * desiredHipAngularPosition.getDoubleValue());
      desiredKneeAngularVelocity.set(0.0);
      desiredKneeAngularAcceleration.set(0.0);
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
      qdHipTrajectory = (qHipTrajectory - desiredHipAngularPosition.getDoubleValue()) / SinggleLeggedSimulation.DT;
      qddHipTrajectory = (qdHipTrajectory - desiredHipAngularVelocity.getDoubleValue()) / SinggleLeggedSimulation.DT;
      desiredHipAngularPosition.set(qHipTrajectory);
      desiredHipAngularVelocity.set(qdHipTrajectory);
      desiredHipAngularAcceleration.set(qddHipTrajectory);

      qKneeTrajectory = -2 * desiredHipAngularPosition.getDoubleValue();
      qdKneeTrajectory = -2 * desiredHipAngularVelocity.getDoubleValue();
      qddKneeTrajectory = -2 * desiredHipAngularAcceleration.getDoubleValue();
      desiredKneeAngularPosition.set(qKneeTrajectory);
      desiredKneeAngularVelocity.set(qdKneeTrajectory);
      desiredKneeAngularAcceleration.set(qddKneeTrajectory);
   }

   private void TrajectoryForCollectingData()
   {
      if (iteration > numOfDataPerTrajectory)
      {
         saveData();
      }
      else if (grfLearning.getDataIdx() == numOfDataPerTrajectory)
      {
         nextTrajectory();
      }
   }

   private void saveData()
   {
      if (robot.getGRFz() == 0)
      {
         System.out.println("no contact point");
         grfLearning.clearData();
         nextTrajectory();
      }
      else
      {
         switch (trajectoryType)
         {
            case POLYNOMIAL:
               if (initPolynomialTrajectory)
               {
                  trajectoryGenerator.nextRandomPolynomialTrajectory(robot.getTime(), robot.getTime() + td, position_z);
                  initPolynomialTrajectory = false;
               }
               previousDesiredHipPosition = desiredHipPosition.getDoubleValue();
               desiredHipPosition.set(trajectoryGenerator.polynomialTrajectory(robot.getTime()));
               desiredHipVelocity.set((desiredHipPosition.getDoubleValue() - previousDesiredHipPosition) / SinggleLeggedSimulation.DT);
               updateTrajectory(desiredHipPosition);
               break;

            case SINWAVE:
               desiredHipPosition.set(trajectoryGenerator.sinwaveTrajectory());
               updateTrajectory(desiredHipPosition);
               break;

            default:
               break;
         }
         grfLearning.saveData();
      }
   }

   private void nextTrajectory()
   {
      System.out.println("=====new Trajectory for Data collecting=====");
      grfLearning.saveFile();
      grfLearning.clearData();

      switch (trajectoryType)
      {
         case POLYNOMIAL:
            trajectoryGenerator.nextRandomPolynomialTrajectory(robot.getTime(), robot.getTime() + td, position_z);
            break;

         case SINWAVE:
            iteration = 0;
            setPointTrajectory(trajectoryGenerator.getSinOffset());
            trajectoryGenerator.nextDelicateSinwaveTrajectory();
            break;

         default:
            break;
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
