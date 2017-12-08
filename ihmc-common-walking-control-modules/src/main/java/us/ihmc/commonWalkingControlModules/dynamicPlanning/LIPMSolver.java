package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class LIPMSolver
{
   private final double mass = 150;
   private final double dt = 0.01;

   private static final double alpha = 1.0;

   private final LIPMDynamics dynamics;
   private final LQCostFunction costFunction;
   private final LQCostFunction terminalCostFunction;

   private final RecyclingArrayList<DenseMatrix64F> stateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> controlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> updatedStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> updatedControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> feedBackGainTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private boolean useDynamicsHessian = false;
   private final boolean debug;

   private final DenseMatrix64F Q = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F R = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F Vxx = new DenseMatrix64F(6, 6);

   private final SimpleDDPSolver<LIPMState> ddpSolver;
   private final DiscreteTimeVaryingTrackingLQRSolver<LIPMState> lqr;

   public LIPMSolver()
   {
      this(false);
   }

   public LIPMSolver(boolean debug)
   {
      this.dynamics = new LIPMDynamics(0.01, 150, 9.81);
      this.costFunction = new LIPMSimpleCostFunction();
      this.terminalCostFunction = new LIPMTerminalCostFunction();
      this.debug = debug;

      ddpSolver = new SimpleDDPSolver<>(dynamics, costFunction, terminalCostFunction);
      lqr = new DiscreteTimeVaryingTrackingLQRSolver<>(dynamics, costFunction, terminalCostFunction);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder gainBuilder = new VariableVectorBuilder(controlSize, stateSize);

      stateTrajectory = new RecyclingArrayList<>(1000, stateBuilder);
      controlTrajectory = new RecyclingArrayList<>(1000, controlBuilder);
      desiredStateTrajectory = new RecyclingArrayList<>(1000, stateBuilder);
      desiredControlTrajectory = new RecyclingArrayList<>(1000, controlBuilder);
      updatedStateTrajectory = new RecyclingArrayList<>(1000, stateBuilder);
      updatedControlTrajectory = new RecyclingArrayList<>(1000, controlBuilder);

      feedBackGainTrajectory = new RecyclingArrayList<>(1000, gainBuilder);
      feedForwardTrajectory = new RecyclingArrayList<>(1000, controlBuilder);

      stateTrajectory.clear();
      controlTrajectory.clear();
      desiredStateTrajectory.clear();
      desiredControlTrajectory.clear();
      updatedStateTrajectory.clear();
      updatedControlTrajectory.clear();

      feedBackGainTrajectory.clear();
      feedForwardTrajectory.clear();
   }

   public void initializeTrajectory(RecyclingArrayList<DenseMatrix64F> desiredZmpTrajectory,
                                 RecyclingArrayList<DenseMatrix64F> desiredCoMTrajectory, DenseMatrix64F initialCoM)
   {
      costFunction.getCostStateHessian(desiredZmpTrajectory.getFirst(), desiredCoMTrajectory.getFirst(), Q);
      costFunction.getCostControlHessian(desiredZmpTrajectory.getFirst(), desiredCoMTrajectory.getFirst(), R);

      setZMPTrajectory(desiredZmpTrajectory, desiredCoMTrajectory, initialCoM);

      ddpSolver.initializeFromLQRSolution(LIPMState.NORMAL, stateTrajectory, controlTrajectory, desiredCoMTrajectory, desiredZmpTrajectory,
                                          lqr.getOptimalFeedbackGainTrajectory(), lqr.getOptimalFeedForwardControlTrajectory());
      ddpSolver.initializeTrajectoriesFromDesireds(initialCoM, desiredCoMTrajectory, desiredZmpTrajectory);

      //ddpSolver.computeTrajectory(initialCoM, desiredCoMTrajectory, desiredZmpTrajectory);
   }

   public void computeTrajectory()
   {
      ddpSolver.computeTrajectory(LIPMState.NORMAL);
      /*
      double cost, cost0;
      cost0 = Double.POSITIVE_INFINITY;

      for (int iter = 0; iter < 1000; iter++)
      {
         backwardPass(stateTrajectory, controlTrajectory);
         cost = forwardPass(stateTrajectory.getFirst(), stateTrajectory, controlTrajectory);

         for (int i = 0; i < updatedControlTrajectory.size(); i++)
         {
            stateTrajectory.get(i).set(updatedStateTrajectory.get(i));
            controlTrajectory.get(i).set(updatedControlTrajectory.get(i));
         }

         if (Math.abs(cost - cost0) / Math.abs(cost0) < 1e-3)
            break;

         cost0 = cost;
      }

*/
   }

   void setZMPTrajectory(RecyclingArrayList<DenseMatrix64F> desiredZmpTrajectory, RecyclingArrayList<DenseMatrix64F> desiredCoMTrajectory, DenseMatrix64F initialCoM)
   {
      int end = desiredCoMTrajectory.size() - 1;

      lqr.setDesiredTrajectories(desiredCoMTrajectory, desiredZmpTrajectory, initialCoM);
      lqr.solveRiccatiEquation(LIPMState.NORMAL, 0, end);
      lqr.computeOptimalTrajectories(LIPMState.NORMAL, 0, end);

      Vxx.set(lqr.getValueHessian());

      feedForwardTrajectory.clear();
      feedBackGainTrajectory.clear();
      stateTrajectory.clear();
      controlTrajectory.clear();
      updatedControlTrajectory.clear();
      updatedStateTrajectory.clear();

      for (int i = 0; i < desiredCoMTrajectory.size(); i++)
      {
         feedBackGainTrajectory.add().set(lqr.getOptimalFeedbackGainTrajectory().get(i));
         feedForwardTrajectory.add().set(lqr.getOptimalFeedForwardControlTrajectory().get(i));
         stateTrajectory.add().zero();
         controlTrajectory.add().zero();
         updatedControlTrajectory.add().zero();
         updatedStateTrajectory.add().zero();

         this.desiredStateTrajectory.add().set(desiredCoMTrajectory.get(i));
         this.desiredControlTrajectory.add().set(desiredZmpTrajectory.get(i));
      }
      stateTrajectory.getAndGrowIfNeeded(0).set(initialCoM);


      /*
      forwardPass(initialCoM, updatedStateTrajectory, updatedControlTrajectory);
      for (int i = 0; i < updatedControlTrajectory.size(); i++)
      {
         stateTrajectory.getAndGrowIfNeeded(i).set(updatedStateTrajectory.get(i));
         controlTrajectory.getAndGrowIfNeeded(i).set(updatedControlTrajectory.get(i));
      }
      */
   }



   public RecyclingArrayList<DenseMatrix64F> getControlTrajectory()
   {
      return ddpSolver.getControlTrajectory();
   }

   public RecyclingArrayList<DenseMatrix64F> getStateTrajectory()
   {
      return ddpSolver.getStateTrajectory();
   }
}
