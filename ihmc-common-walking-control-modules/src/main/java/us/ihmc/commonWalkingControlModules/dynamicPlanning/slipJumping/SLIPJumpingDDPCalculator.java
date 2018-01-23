package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs.SLIPDesiredTrackingCost;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs.SLIPModelForceTrackingCost;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs.SLIPRegularizationCost;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs.SLIPTerminalCost;
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
   private final DiscreteSequence constantSequence;

   private final SimpleDDPSolver<SLIPState> ddpSolver;

   private final double mass;
   private final double gravityZ;
   private final double nominalHeight;

   private final DenseMatrix64F initialState;

   private final CompositeLQCostFunction<SLIPState> regularCostFunction = new CompositeLQCostFunction<>();
   private final LQTrackingCostFunction<SLIPState> terminalCostFunction = new SLIPTerminalCost();

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

      LQCostFunction<SLIPState> slipModelTrackingCost = new SLIPModelForceTrackingCost(mass, gravityZ);
      LQCostFunction<SLIPState> slipRegularizationCost = new SLIPRegularizationCost();

      LQTrackingCostFunction<SLIPState> slipDesiredTrackingCost = new SLIPDesiredTrackingCost();

      regularCostFunction.addLQCostFunction(slipModelTrackingCost);
      regularCostFunction.addLQCostFunction(slipRegularizationCost);
      regularCostFunction.addLQTrackingCostFunction(slipDesiredTrackingCost);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      initialState = new DenseMatrix64F(stateSize, 1);

      optimalSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      desiredSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      constantSequence = new DiscreteSequence(constantSize);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
   }


   public void initialize(DenseMatrix64F currentState, FramePoint3D firstSupport, FramePoint3D apexPoint, FramePoint3D secondSupport,
                          double firstStanceDuration, double flightDuration, double secondStanceDuration, double nominalInitialStiffness,
                          double nominalFinalStiffness)
   {
      initialState.set(currentState);

      int numberOfInitialTimeSteps = (int) Math.floor(firstStanceDuration / deltaT);
      int numberOfFlightTimeSteps = (int) Math.floor(flightDuration / deltaT);
      int numberOfFinalTimeSteps = (int) Math.floor(secondStanceDuration / deltaT);

      double nominalFirstLength = nominalHeight + mass * gravityZ / nominalInitialStiffness;
      double nominalSecondLength = nominalHeight + mass * gravityZ / nominalFinalStiffness;

      double modifiedDeltaT = firstStanceDuration / numberOfInitialTimeSteps;
      dynamics.setTimeStepSize(modifiedDeltaT);

      dynamicStates.add(SLIPState.STANCE);
      startIndices.add(0);
      endIndices.add(numberOfInitialTimeSteps - 1);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(null);

      dynamicStates.add(SLIPState.FLIGHT);
      startIndices.add(numberOfInitialTimeSteps);
      endIndices.add(numberOfInitialTimeSteps + numberOfFlightTimeSteps - 1);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(null);

      dynamicStates.add(SLIPState.STANCE);
      startIndices.add(numberOfInitialTimeSteps + numberOfFlightTimeSteps);
      endIndices.add(numberOfInitialTimeSteps + numberOfFlightTimeSteps + numberOfFinalTimeSteps - 1);
      costFunctions.add(regularCostFunction);
      terminalCostFunctions.add(terminalCostFunction);

      int totalSize = numberOfInitialTimeSteps + numberOfFlightTimeSteps + numberOfFinalTimeSteps;
      desiredSequence.setLength(totalSize);
      optimalSequence.setLength(totalSize);
      constantSequence.setLength(totalSize);

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

         DenseMatrix64F constants = constantSequence.get(i);
         constants.set(zF, firstSupport.getZ());
         constants.set(nominalLength, nominalFirstLength);
         //constants.set(nominalLength, nominalHeight);
      }

      for (int i = numberOfInitialTimeSteps; i < numberOfInitialTimeSteps + numberOfFlightTimeSteps; i++)
      {
         DenseMatrix64F desiredState = desiredSequence.getState(i);
         desiredState.set(x, apexPoint.getX());
         desiredState.set(y, apexPoint.getY());
         desiredState.set(z, apexPoint.getZ());

         DenseMatrix64F desiredControl = desiredSequence.getControl(i);
         desiredControl.set(fz, 0.0);
         desiredControl.set(xF, firstSupport.getX());
         desiredControl.set(yF, firstSupport.getY());
         desiredControl.set(k, 0.0);
      }

      for (int i = numberOfInitialTimeSteps + numberOfFlightTimeSteps; i < totalSize; i++)
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

         DenseMatrix64F constants = constantSequence.get(i);
         constants.set(zF, secondSupport.getZ());
         constants.set(nominalLength, nominalSecondLength);
         //constants.set(nominalLength, nominalHeight);
      }

      ddpSolver.initializeSequencesFromDesireds(currentState, desiredSequence, constantSequence);
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
