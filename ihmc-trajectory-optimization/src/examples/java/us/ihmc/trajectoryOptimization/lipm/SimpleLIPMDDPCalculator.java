package us.ihmc.trajectoryOptimization.lipm;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.trajectoryOptimization.*;
import us.ihmc.trajectoryOptimization.DDPSolver;

import java.util.List;

public class SimpleLIPMDDPCalculator
{
   private final DiscreteHybridDynamics<DefaultDiscreteState> dynamics;

   private double deltaT;
   private double modifiedDeltaT;

   private final DiscreteOptimizationTrajectory desiredTrajectory;
   private final DiscreteSequence constants;

   private int numberOfTimeSteps;

   private final LQTrackingCostFunction<DefaultDiscreteState> costFunction;
   private final LQTrackingCostFunction<DefaultDiscreteState> terminalCostFunction;
   private final DDPSolver<DefaultDiscreteState> ddpSolver;

   public SimpleLIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new SimpleLIPMDynamics(deltaT, 1.0, gravityZ);
      costFunction = new SimpleLIPMSimpleCostFunction(); // discrete, so we need to take that into account
      terminalCostFunction = new SimpleLIPMTerminalCostFunction();
      this.deltaT = deltaT;

      ddpSolver = new DDPSolver<>(dynamics, true);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      desiredTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
      constants = new DiscreteSequence(constantSize);
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
      int numberOfTimeSteps = copPositions.size();
      modifiedDeltaT = duration / numberOfTimeSteps;
      dynamics.setTimeStepSize(modifiedDeltaT);
      desiredTrajectory.setTrajectoryDuration(0, duration, deltaT);
      constants.setLength(desiredTrajectory.size());

      double time = 0.0;
      tempPoint.setIncludingFrame(copPositions.get(0));
      tempPoint.setIncludingFrame(copVelocities.get(0));
      DMatrixRMaj desiredState = desiredTrajectory.getState(0);

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempVector.getX());
      desiredState.set(3, tempVector.getY());

      DMatrixRMaj desiredControl = desiredTrajectory.getControl(0);
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());

      time += modifiedDeltaT;


      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         tempPoint.setIncludingFrame(copPositions.get(i));
         tempPoint.setIncludingFrame(copVelocities.get(i));
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

      ddpSolver.initializeSequencesFromDesireds(currentState, desiredTrajectory, constants);
      //ddpSolver.solveBackwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      //ddpSolver.solveForwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      //ddpSolver.initializeDDPWithLQRSolution();
   }

   public double getDT()
   {
      return modifiedDeltaT;
   }
}
