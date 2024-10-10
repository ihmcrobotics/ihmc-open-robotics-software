package us.ihmc.trajectoryOptimization.lipm;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.trajectoryOptimization.*;
import us.ihmc.trajectoryOptimization.SimpleDDPSolver;
import us.ihmc.trajectoryOptimization.DiscreteTimeVaryingTrackingLQRSolver;
import us.ihmc.trajectoryOptimization.LQRSolverInterface;

import java.util.List;

public class LIPMDDPCalculator
{
   private final DiscreteHybridDynamics<DefaultDiscreteState> dynamics;

   private double deltaT;
   private double modifiedDeltaT;

   private final DiscreteOptimizationTrajectory desiredTrajectory;
   private final DiscreteOptimizationTrajectory optimalTrajectory;
   private final DiscreteSequence constantSequence;

   private final SimpleDDPSolver<DefaultDiscreteState> ddpSolver;
   private final LQRSolverInterface<DefaultDiscreteState> lqrSolver;

   private final LQTrackingCostFunction<DefaultDiscreteState> costFunction = new LIPMSimpleCostFunction();
   private final LQTrackingCostFunction<DefaultDiscreteState> terminalCostFunction = new LIPMTerminalCostFunction();

   private double mass;
   private double gravityZ;

   public LIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new LIPMDynamics(deltaT, mass, gravityZ);
      this.deltaT = deltaT;
      this.mass = mass;
      this.gravityZ = gravityZ;

      ddpSolver = new SimpleDDPSolver<>(dynamics, true);
      lqrSolver = new DiscreteTimeVaryingTrackingLQRSolver<>(dynamics, costFunction, terminalCostFunction);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      desiredTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
      optimalTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
      constantSequence = new DiscreteSequence(constantSize);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
      this.modifiedDeltaT = deltaT;
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void initialize(DMatrixRMaj currentState, List<FramePoint3DReadOnly> copPositions, List<FrameVector3DReadOnly> copVelocities, double duration)
   {
      modifiedDeltaT = duration / copPositions.size();
      int numberOfTimeSteps = copPositions.size();
      dynamics.setTimeStepSize(modifiedDeltaT);
      desiredTrajectory.setTrajectoryDuration(0, duration, deltaT);
      optimalTrajectory.setTrajectoryDuration(0, duration, deltaT);
      constantSequence.setLength(desiredTrajectory.size());

      double height = currentState.get(2);

      tempPoint.setIncludingFrame(copPositions.get(0));
      tempVector.setIncludingFrame(copVelocities.get(0));

      DMatrixRMaj desiredState = desiredTrajectory.getState(0);

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempPoint.getZ() + height);
      desiredState.set(3, tempVector.getX());
      desiredState.set(4, tempVector.getY());
      desiredState.set(5, tempVector.getZ());

      DMatrixRMaj desiredControl = desiredTrajectory.getControl(0);
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());
      desiredControl.set(2, mass * gravityZ);

      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         tempPoint.setIncludingFrame(copPositions.get(i));
         tempVector.setIncludingFrame(copVelocities.get(i));
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
      }

      lqrSolver.setDesiredSequence(desiredTrajectory, constantSequence, currentState);
      lqrSolver.solveRiccatiEquation(DefaultDiscreteState.DEFAULT, 0, desiredTrajectory.size() - 1);
      lqrSolver.computeOptimalSequences(DefaultDiscreteState.DEFAULT, 0, desiredTrajectory.size() - 1);
      lqrSolver.getOptimalSequence(optimalTrajectory);

      ddpSolver.initializeFromLQRSolution(DefaultDiscreteState.DEFAULT, costFunction, optimalTrajectory, desiredTrajectory, constantSequence,
                                          lqrSolver.getOptimalFeedbackGainSequence(), lqrSolver.getOptimalFeedForwardControlSequence());
   }

   public int solve()
   {
      int iterations = ddpSolver.computeSequence(DefaultDiscreteState.DEFAULT, costFunction, terminalCostFunction);
      optimalTrajectory.set(ddpSolver.getOptimalSequence());
      return iterations;
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
