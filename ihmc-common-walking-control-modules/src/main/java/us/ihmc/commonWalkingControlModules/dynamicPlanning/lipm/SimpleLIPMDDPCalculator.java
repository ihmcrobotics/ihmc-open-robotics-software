package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.trajectoryOptimization.*;

public class SimpleLIPMDDPCalculator
{
   private final DiscreteHybridDynamics<DefaultDiscreteState> dynamics;

   private double deltaT;
   private double modifiedDeltaT;

   private final DiscreteOptimizationTrajectory desiredTrajectory;

   private int numberOfTimeSteps;

   private final DDPSolver<DefaultDiscreteState> ddpSolver;

   public SimpleLIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new SimpleLIPMDynamics(deltaT, 1.0, gravityZ);
      LQCostFunction costFunction = new SimpleLIPMSimpleCostFunction(); // discrete, so we need to take that into account
      LQCostFunction terminalCostFunction = new SimpleLIPMTerminalCostFunction();
      this.deltaT = deltaT;

      ddpSolver = new DDPSolver<>(dynamics, costFunction, terminalCostFunction, true);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      desiredTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
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

      double time = 0.0;
      copDesiredPlan.update(time, tempPoint, tempVector);
      DenseMatrix64F desiredState = desiredTrajectory.getState(0);

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempVector.getX());
      desiredState.set(3, tempVector.getY());

      DenseMatrix64F desiredControl = desiredTrajectory.getControl(0);
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());

      time += modifiedDeltaT;


      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         copDesiredPlan.update(time, tempPoint, tempVector);
         desiredState = desiredTrajectory.getState(i);

         desiredState.set(0, tempPoint.getX());
         desiredState.set(1, tempPoint.getY());
         desiredState.set(2, tempVector.getX());
         desiredState.set(3, tempVector.getY());

         desiredControl = desiredTrajectory.getControl(i);
         desiredControl.set(0, tempPoint.getX());
         desiredControl.set(1, tempPoint.getY());

         time += modifiedDeltaT;
      }

      ddpSolver.initializeTrajectoriesFromDesireds(currentState, desiredTrajectory);
      //ddpSolver.solveBackwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      //ddpSolver.solveForwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      //ddpSolver.initializeDDPWithLQRSolution();
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
}
