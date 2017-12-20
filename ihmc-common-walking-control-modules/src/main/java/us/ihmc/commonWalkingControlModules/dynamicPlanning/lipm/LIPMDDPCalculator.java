package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.trajectoryOptimization.*;

public class LIPMDDPCalculator
{
   private final DiscreteHybridDynamics<DefaultDiscreteState> dynamics;

   private double deltaT;
   private double modifiedDeltaT;

   private final DiscreteOptimizationTrajectory desiredTrajectory;
   private final DiscreteOptimizationTrajectory optimalTrajectory;

   private int numberOfTimeSteps;

   private final SimpleDDPSolver<DefaultDiscreteState> ddpSolver;
   private final LQRSolverInterface<DefaultDiscreteState> lqrSolver;

   private double mass;
   private double gravityZ;

   public LIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new LIPMDynamics(deltaT, mass, gravityZ);
      this.deltaT = deltaT;
      this.mass = mass;
      this.gravityZ = gravityZ;

      LQCostFunction costFunction = new LIPMSimpleCostFunction();
      LQCostFunction terminalCostFunction = new LIPMTerminalCostFunction();
      ddpSolver = new SimpleDDPSolver<>(dynamics, costFunction, terminalCostFunction, true);
      lqrSolver = new DiscreteTimeVaryingTrackingLQRSolver<>(dynamics, costFunction, terminalCostFunction);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      desiredTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
      optimalTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
      this.modifiedDeltaT = deltaT;
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void initialize(DenseMatrix64F currentState, SegmentedFrameTrajectory3D copDesiredPlan)
   {
      modifiedDeltaT = computeDeltaT(copDesiredPlan.getFinalTime());
      dynamics.setTimeStepSize(modifiedDeltaT);
      desiredTrajectory.setTrajectoryDuration(0, copDesiredPlan.getFinalTime(), deltaT);
      optimalTrajectory.setTrajectoryDuration(0, copDesiredPlan.getFinalTime(), deltaT);

      double height = currentState.get(2);

      double time = 0.0;

      copDesiredPlan.update(time, tempPoint, tempVector);
      DenseMatrix64F desiredState = desiredTrajectory.getState(0);

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempPoint.getZ() + height);
      desiredState.set(3, tempVector.getX());
      desiredState.set(4, tempVector.getY());
      desiredState.set(5, tempVector.getZ());

      DenseMatrix64F desiredControl = desiredTrajectory.getControl(0);
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());
      desiredControl.set(2, mass * gravityZ);

      time += modifiedDeltaT;

      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         copDesiredPlan.update(time, tempPoint, tempVector);
         desiredState = desiredTrajectory.getState(i);

         desiredState.set(0, tempPoint.getX());
         desiredState.set(1, tempPoint.getY());
         desiredState.set(2, tempPoint.getZ() + height);
         desiredState.set(3, tempVector.getX());
         desiredState.set(4, tempVector.getY());
         desiredState.set(5, tempVector.getZ());

         desiredControl = desiredTrajectory.getControl(i);
         desiredControl.set(0, tempPoint.getX());
         desiredControl.set(1, tempPoint.getY());
         desiredControl.set(2, mass * gravityZ);

         time += modifiedDeltaT;
      }

      lqrSolver.setDesiredTrajectory(desiredTrajectory, currentState);
      lqrSolver.solveRiccatiEquation(DefaultDiscreteState.DEFAULT, 0, desiredTrajectory.size() - 1);
      lqrSolver.computeOptimalTrajectories(DefaultDiscreteState.DEFAULT, 0, desiredTrajectory.size() - 1);
      lqrSolver.getOptimalTrajectory(optimalTrajectory);

      ddpSolver.initializeFromLQRSolution(DefaultDiscreteState.DEFAULT, optimalTrajectory, desiredTrajectory, lqrSolver.getOptimalFeedbackGainTrajectory(),
                                          lqrSolver.getOptimalFeedForwardControlTrajectory());
   }

   public int solve()
   {
      int iterations = ddpSolver.computeTrajectory(DefaultDiscreteState.DEFAULT);
      optimalTrajectory.set(ddpSolver.getOptimalTrajectory());
      return iterations;
   }

   private double computeDeltaT(double trajectoryLength)
   {
      numberOfTimeSteps = (int) Math.floor(trajectoryLength / deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   public double getDT()
   {
      return modifiedDeltaT;
   }

   public DiscreteOptimizationTrajectory getOptimalTrajectory()
   {
      return optimalTrajectory;
   }

   public double getValue()
   {
      return 0.0;
   }
}
