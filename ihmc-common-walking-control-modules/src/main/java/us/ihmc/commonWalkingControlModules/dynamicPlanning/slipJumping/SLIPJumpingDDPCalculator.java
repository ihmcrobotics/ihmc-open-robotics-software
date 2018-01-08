package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDynamics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMSimpleCostFunction;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMTerminalCostFunction;
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
   private double modifiedDeltaT;

   private final DiscreteOptimizationSequence optimalSequence;
   private final DiscreteOptimizationSequence desiredSequence;

   private final SimpleDDPSolver<SLIPState> ddpSolver;

   private final double mass;
   private final double gravityZ;
   private final double nominalHeight;

   private double firstStanceDuration;
   private double flightDuration;
   private double secondStanceDuration;

   private final DenseMatrix64F initialState;

   private final List<SLIPState> dynamicStates = new ArrayList<>();
   private final TIntArrayList startIndices = new TIntArrayList();
   private final TIntArrayList endIndices = new TIntArrayList();

   public SLIPJumpingDDPCalculator(double deltaT, double mass, double nominalHeight, double gravityZ)
   {
      this.dynamics = new SimpleReactionDynamics(deltaT, mass, gravityZ);
      this.deltaT = deltaT;
      this.mass = mass;
      this.nominalHeight = nominalHeight;
      this.gravityZ = gravityZ;

      LQTrackingCostFunction costFunction = new LIPMSimpleCostFunction();
      LQTrackingCostFunction terminalCostFunction = new LIPMTerminalCostFunction();
      ddpSolver = new SimpleDDPSolver<>(dynamics, costFunction, terminalCostFunction, true);


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
      this.modifiedDeltaT = deltaT;
   }


   public void initialize(DenseMatrix64F currentState, FramePoint3D firstSupport, FramePoint3D secondSupport,
                          double firstStanceDuration, double flightDuration, double secondStanceDuration)
   {
      // TODO do something with the current state
      this.firstStanceDuration = firstStanceDuration;
      this.flightDuration = flightDuration;
      this.secondStanceDuration = secondStanceDuration;

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

      dynamicStates.add(SLIPState.FLIGHT);
      startIndices.add(numberOfInitialTimeSteps - 1);
      endIndices.add(numberOfInitialTimeSteps);

      dynamicStates.add(SLIPState.STANCE);
      startIndices.add(numberOfInitialTimeSteps);
      endIndices.add(numberOfInitialTimeSteps + numberOfFinalTimeSteps - 1);

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

   }

   public int solve()
   {
      return ddpSolver.computeSequence(dynamicStates, startIndices, endIndices);
   }

   private double computeDeltaT(double trajectoryLength)
   {
      int numberOfTimeSteps = (int) Math.floor(trajectoryLength / deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   public double getValue()
   {
      return 0.0;
   }
}
