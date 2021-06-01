package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCSolutionInspection;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class CoMTrajectoryModelPredictiveController extends EuclideanModelPredictiveController
{
   protected static final boolean debug = false;

   final LinearMPCQPSolver qpSolver;

   private final LinearMPCIndexHandler indexHandler;
   private final LinearMPCSolutionInspection solutionInspection;

   private final IntUnaryOperator firstVariableIndex;

   public CoMTrajectoryModelPredictiveController(double mass, double gravityZ, double nominalCoMHeight, double dt, YoRegistry parentRegistry)
   {
      this(new LinearMPCIndexHandler(numberOfBasisVectorsPerContactPoint), mass, gravityZ, nominalCoMHeight, dt, parentRegistry);
   }

   public CoMTrajectoryModelPredictiveController(LinearMPCIndexHandler indexHandler,
                                                 double mass,
                                                 double gravityZ,
                                                 double nominalCoMHeight,
                                                 double dt,
                                                 YoRegistry parentRegistry)
   {
      super(indexHandler, mass, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      firstVariableIndex = indexHandler::getComCoefficientStartIndex;

      qpSolver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, registry);

      if (debug)
         solutionInspection = new LinearMPCSolutionInspection(indexHandler, gravityZ);
      else
         solutionInspection = null;

      parentRegistry.addChild(registry);
   }

   @Override
   protected void initializeIndexHandler()
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();
      indexHandler.initialize(planningWindow);
   }

   @Override
   protected void resetActiveSet()
   {
      qpSolver.notifyResetActiveSet();
      qpSolver.resetRateRegularization();
   }

   @Override
   protected DMatrixRMaj solveQP()
   {
      qpSolver.initialize();
      qpSolver.submitMPCCommandList(mpcCommands);

      qpSolver.setUseWarmStart(useWarmStart.getBooleanValue());
      if (useWarmStart.getBooleanValue())
      {
         assembleActiveSet(firstVariableIndex);
         qpSolver.setPreviousSolution(previousSolution);
         qpSolver.setActiveInequalityIndices(activeInequalityConstraints);
         qpSolver.setActiveLowerBoundIndices(activeUpperBoundConstraints);
         qpSolver.setActiveUpperBoundIndices(activeLowerBoundConstraints);
      }

      if (!qpSolver.solve())
      {
         LogTools.info("Failed to find solution");
         extractNewActiveSetData(false, qpSolver, firstVariableIndex);

         return null;
      }

      DMatrixRMaj solutionCoefficients = qpSolver.getSolution();

      if (solutionInspection != null)
         solutionInspection.inspectSolution(mpcCommands, solutionCoefficients);

      extractNewActiveSetData(true, qpSolver, firstVariableIndex);

      return solutionCoefficients;
   }


}
