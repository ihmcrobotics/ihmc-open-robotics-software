package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.trajectoryOptimization.*;
import us.ihmc.trajectoryOptimization.SimpleDDPSolver;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPJumpingDDPCalculator
{
   private final SimpleReactionDynamics dynamics;

   private double deltaT;

   private final DiscreteOptimizationSequence optimalSequence;
   private final DiscreteOptimizationSequence desiredSequence;

   private final SimpleDDPSolver<SLIPState> ddpSolver;

   private final double mass;
   private final double gravityZ;
   private final double nominalHeight;

   private final DenseMatrix64F initialState;

   private final CompositeLQCostFunction<SLIPState> regularCostFunction = new CompositeLQCostFunction<>();
   private final LQTrackingCostFunction<SLIPState> terminalCostFunction = new SLIPTerminalCostFunction();

   private final List<SLIPState> dynamicStates = new ArrayList<>();
   private final List<LQTrackingCostFunction<SLIPState>> costFunctions = new ArrayList<>();
   private final List<LQTrackingCostFunction<SLIPState>> terminalCostFunctions = new ArrayList<>();
   private final TIntArrayList startIndices = new TIntArrayList();
   private final TIntArrayList endIndices = new TIntArrayList();

   public SLIPJumpingDDPCalculator(double deltaT, double mass, double nominalHeight, double gravityZ)
   {
      this.dynamics = new SimpleReactionDynamics(deltaT, mass, gravityZ);
      this.deltaT = deltaT;
      this.mass = mass;
      this.nominalHeight = nominalHeight;
      this.gravityZ = gravityZ;

      ddpSolver = new SimpleDDPSolver<>(dynamics, true);

      LQCostFunction<SLIPState> slipModelTrackingCost = new SLIPModelTrackingCost(mass, nominalHeight, gravityZ);
      LQCostFunction<SLIPState> slipRegularizationCost = new SLIPRegularizationCostFunction();

      LQTrackingCostFunction<SLIPState> slipDesiredTrackingCost = new SLIPDesiredTrackingCostFunction();
      LQTrackingCostFunction<SLIPState> slipStateChangeCost = new SLIPStateChangeCostFunction();

      regularCostFunction.addLQCostFunction(slipModelTrackingCost);
      regularCostFunction.addLQCostFunction(slipRegularizationCost);
      regularCostFunction.addLQTrackingCostFunction(slipDesiredTrackingCost);
      regularCostFunction.addLQTrackingCostFunction(slipStateChangeCost);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      initialState = new DenseMatrix64F(stateSize, 1);

      optimalSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      desiredSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
   }


   public void initialize(DenseMatrix64F currentState, FramePoint3D firstSupport, FramePoint3D secondSupport,
                          double firstStanceDuration, double flightDuration, double secondStanceDuration)
   {
      // TODO do something with the current state
      initialState.set(currentState);

      double nominalInitialStiffness = 4.0 * Math.PI * Math.PI * mass / (firstStanceDuration  * firstStanceDuration);
      int numberOfInitialTimeSteps = (int) Math.floor(firstStanceDuration / deltaT);

      double nominalFinalStiffness = 4.0 * Math.PI * Math.PI * mass / (secondStanceDuration * secondStanceDuration);
      int numberOfFinalTimeSteps = (int) Math.floor(secondStanceDuration / deltaT);

      double modifiedDeltaT = firstStanceDuration / numberOfInitialTimeSteps;
      dynamics.setTimeStepSize(modifiedDeltaT);
      dynamics.setFlightDuration(flightDuration);

      dynamicStates.add(SLIPState.STANCE);
      startIndices.add(0);
      endIndices.add(numberOfInitialTimeSteps - 1);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(null);

      dynamicStates.add(SLIPState.FLIGHT);
      startIndices.add(numberOfInitialTimeSteps);
      endIndices.add(numberOfInitialTimeSteps);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(null);

      dynamicStates.add(SLIPState.STANCE);
      startIndices.add(numberOfInitialTimeSteps + 1);
      endIndices.add(numberOfInitialTimeSteps + numberOfFinalTimeSteps - 1);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(terminalCostFunction);

      desiredSequence.setLength(numberOfInitialTimeSteps + numberOfFinalTimeSteps);
      optimalSequence.setLength(numberOfInitialTimeSteps + numberOfFinalTimeSteps);

      for (int i = 0; i < numberOfInitialTimeSteps; i++)
      {
         DenseMatrix64F desiredState = desiredSequence.getState(i);
         desiredState.set(x, firstSupport.getX());
         desiredState.set(y, firstSupport.getY());
         desiredState.set(z, firstSupport.getZ() + nominalHeight);

         DenseMatrix64F desiredControl = desiredSequence.getControl(i);
         desiredControl.set(fz, mass * gravityZ);
         desiredControl.set(xF, firstSupport.getX());
         desiredControl.set(yF, firstSupport.getY());
         desiredControl.set(k, nominalInitialStiffness);
      }

      for (int i = numberOfInitialTimeSteps; i < numberOfInitialTimeSteps + numberOfFinalTimeSteps; i++)
      {
         DenseMatrix64F desiredState = desiredSequence.getState(i);
         desiredState.set(x, secondSupport.getX());
         desiredState.set(y, secondSupport.getY());
         desiredState.set(z, secondSupport.getZ() + nominalHeight);

         DenseMatrix64F desiredControl = desiredSequence.getControl(i);
         desiredControl.set(fz, mass * gravityZ);
         desiredControl.set(xF, secondSupport.getX());
         desiredControl.set(yF, secondSupport.getY());
         desiredControl.set(k, nominalFinalStiffness);
      }

      ddpSolver.initializeSequencesFromDesireds(currentState, desiredSequence);
      optimalSequence.set(desiredSequence);
   }

   public int solve()
   {
      return ddpSolver.computeSequence(dynamicStates, costFunctions, terminalCostFunctions, startIndices, endIndices);
   }

   public void singleSolve()
   {
      ddpSolver.computeOnePass(dynamicStates, costFunctions, terminalCostFunctions, startIndices, endIndices);
      optimalSequence.set(ddpSolver.getOptimalSequence());
   }

   private double computeDeltaT(double trajectoryLength)
   {
      int numberOfTimeSteps = (int) Math.floor(trajectoryLength / deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   public DiscreteOptimizationSequence getOptimalSequence()
   {
      return optimalSequence;
   }

   public double getValue()
   {
      return 0.0;
   }
}
