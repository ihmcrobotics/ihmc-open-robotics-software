package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SinggleLeggedMPCController implements RobotController
{
   private final String name = "SinggleLeggedMPCController";
   private final YoRegistry registry = new YoRegistry(name);
   private SinggleLeggedRobot robot;
   private MPCSolversForSinggleLeggedRobot MPCsolver;
   private SinggleLeggedTrajectoryGenerator trajectoryGenerator;

   private YoDouble desiredHipPosition;
   private YoDouble desiredHipVelocity;
   private YoDouble currentHipPosition;
   private YoDouble currentHipVelocity;

   
   private int iteration;
   
   private double initialPosition;
   private double desiredPosition;
   private double previousHipVelocity;
   private double calculatedGRFz;

   private MatrixForML activeTorque;
   private MatrixForML desiredPositionTrajectorySequence;
   private MatrixForML desiredTrajectorySequence;

   private double GRFpenalty;
   private int MPCHorizon;
   private MatrixForML positionTrajectory;
   private MatrixForML velocityTrajectory;

   private long startedTime;
   private long finishedTime;
   private long elapsedTime;
   private long totalElapsedTime = 0;
   private double dT = SinggleLeggedSimulation.DT;

   private enum trajectoryType
   {
      SETPOINT, SINWAVE
   };

   private trajectoryType trajectory;

   public SinggleLeggedMPCController(SinggleLeggedRobot robot)
   {
      this.robot = robot;
      MPCsolver = new MPCSolversForSinggleLeggedRobot();
      trajectoryGenerator = new SinggleLeggedTrajectoryGenerator();
      desiredHipPosition = new YoDouble("DesiredHipPos", registry);
      desiredHipVelocity = new YoDouble("DesiredHipVel", registry);
      currentHipPosition = new YoDouble("CurrentHipPosition", registry);
      currentHipVelocity = new YoDouble("CurrentHipVelocity", registry);
      activeTorque = new MatrixForML(2, 1);
      positionTrajectory = new MatrixForML(10000, 1);
      velocityTrajectory = new MatrixForML(10000, 1);
      trajectory = trajectoryType.SETPOINT;

      initialize();
   }

   @Override
   public void initialize()
   {
      iteration = -1;
      initialPosition = 0.425;
      currentHipPosition.set(initialPosition);
      desiredHipPosition.set(initialPosition);

      // For MPC
      GRFpenalty = 0.000001;
      MPCHorizon = 10;
      desiredPositionTrajectorySequence = new MatrixForML(MPCHorizon, 1);
      desiredTrajectorySequence = new MatrixForML(MPCHorizon, 2);

      switch (trajectory)
      {
         case SETPOINT:
            initializeSetPointTrajectory(0.225, 0.20);
            break;
         case SINWAVE:
            initializeSinWaveTrajectory(0.425, 0.1, 0.5);
            break;
         default:
            System.out.println("Please check the selected trajectory.");
            break;
      }
   }

   private void initializeSetPointTrajectory(double targetPosition, double targetTime)
   {
      desiredPosition = targetPosition;

      for (int i = 0; i < 10000; i++)
      {
         if (i == 0)
         {
            positionTrajectory.set(i, 0, initialPosition + (desiredPosition - initialPosition) * (i / (targetTime / dT)));
            velocityTrajectory.set(i, 0, (positionTrajectory.getDoubleValue(1, 0) - positionTrajectory.getDoubleValue(0, 0)) / dT);
         }
         else if (i < (int) (targetTime / dT))
         {
            positionTrajectory.set(i, 0, initialPosition + (desiredPosition - initialPosition) * (i / (targetTime / dT)));
            velocityTrajectory.set(i, 0, (positionTrajectory.getDoubleValue(i, 0) - positionTrajectory.getDoubleValue(i - 1, 0)) / dT);
         }
         else
         {
            positionTrajectory.set(i, 0, desiredPosition);
         }
      }
   }

   private void initializeSinWaveTrajectory(double offset, double amplitude, double frequency)
   {
      trajectoryGenerator.nextSinwaveTrajectory(offset, amplitude, frequency);
      for (int i = 0; i < 10000; i++)
      {
         desiredPosition = trajectoryGenerator.sinwaveTrajectory();
         positionTrajectory.set(i, 0, desiredPosition);
         if (i == 0)
         {
            velocityTrajectory.set(i, 0, (positionTrajectory.getDoubleValue(1, 0) - positionTrajectory.getDoubleValue(0, 0)) / dT);
         }
         else
         {
            velocityTrajectory.set(i, 0, (positionTrajectory.getDoubleValue(i, 0) - positionTrajectory.getDoubleValue(i - 1, 0)) / dT);
         }
      }
   }

   @Override
   public void doControl()
   {
      iteration++;
      if (iteration > 0)
      {
         System.out.println("\nCurrent iteration : " + iteration);
         updateDesiredTrajectorySequence();
         updateCurrentHipVelocity();
         updateCurrentHipPosition();
         updateTrajectory();

         tic();
         calculateDesiredGRFz(); // MPC
         toc();

         computeTorques();
         setTorques();
         updatePreviousHipVelocity();
      }
   }

   private void updateDesiredPositionTrajectorySequence() // this method is required when using MPC gradient descent ver1 or LM method ver1.
   {
      for (int i = 0; i < MPCHorizon; i++)
      {
         desiredPositionTrajectorySequence.set(i, 0, positionTrajectory.getDoubleValue(i + iteration, 0));
      }
   }

   private void updateDesiredTrajectorySequence() // this method is required when using MPC ver2.
   {
      for (int i = 0; i < MPCHorizon; i++)
      {
         desiredTrajectorySequence.set(i, 0, positionTrajectory.getDoubleValue(i + iteration, 0));
         desiredTrajectorySequence.set(i, 1, velocityTrajectory.getDoubleValue(i + iteration, 0));
      }
   }

   private void updatePreviousHipVelocity()
   {
      previousHipVelocity = currentHipVelocity.getDoubleValue();
   }

   private void updateCurrentHipVelocity()
   {
      currentHipVelocity.set(robot.getHipSlideVelocity());
   }

   private void updateCurrentHipPosition()
   {
      currentHipPosition.set((robot.UPPER_LEG_H * Math.cos(robot.getHipAngularPosition())
            + robot.LOWER_LEG_H * Math.cos(robot.getHipAngularPosition() + robot.getKneeAngularPosition())));

   }

   private void updateTrajectory()
   {
      desiredHipVelocity.set(velocityTrajectory.getDoubleValue(iteration, 0));
      desiredHipPosition.set(positionTrajectory.getDoubleValue(iteration, 0));
   }

   private void calculateDesiredGRFz()
   {
      calculatedGRFz = MPCsolver.GradientDescentMethodVer2(MPCHorizon,
                                                           currentHipPosition.getDoubleValue(),
                                                           currentHipVelocity.getDoubleValue(),
                                                           previousHipVelocity,
                                                           SinggleLeggedSimulation.DT,
                                                           robot.HIP_MASS + robot.UPPER_LEG_MASS + robot.LOWER_LEG_MASS,
                                                           GRFpenalty,
                                                           desiredTrajectorySequence);
   }

   private void computeTorques()
   {
      double l1 = robot.UPPER_LEG_H;
      double l2 = robot.LOWER_LEG_H;
      double th1 = robot.getHipAngularPosition();
      double th2 = robot.getKneeAngularPosition();
      double dz_dth1, dz_dth2;
      double hipTau, kneeTau;
      
      // TODO make getJacobian() method and change it to matrix multiplication.
      dz_dth1 = -l1 * Math.sin(th1) - l2 * Math.sin(th1 + th2);
      dz_dth2 = -l2 * Math.sin(th1 + th2);

      hipTau = dz_dth1 * calculatedGRFz;
      kneeTau = dz_dth2 * calculatedGRFz;

      
      activeTorque.set(0,0,hipTau);
      activeTorque.set(1,0,kneeTau);
   }

   private void setTorques()
   {
      robot.setHipTorque(activeTorque.getDoubleValue(0, 0));
      robot.setKneeTorque(activeTorque.getDoubleValue(1, 0));
   }

   private void tic()
   {
      startedTime = System.nanoTime();
   }

   private void toc()
   {
      finishedTime = System.nanoTime();
      elapsedTime = finishedTime - startedTime;
      totalElapsedTime += elapsedTime;
      System.out.println("It takes " + elapsedTime * 1e-9 + "seconds.");
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
