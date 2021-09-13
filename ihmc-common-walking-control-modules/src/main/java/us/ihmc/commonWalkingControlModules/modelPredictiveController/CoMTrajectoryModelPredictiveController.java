package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class CoMTrajectoryModelPredictiveController extends EuclideanModelPredictiveController
{
   protected static final boolean debug = false;

   final LinearMPCQPSolver qpSolver;

   private final LinearMPCIndexHandler indexHandler;

   private final IntUnaryOperator firstVariableIndex;

   public CoMTrajectoryModelPredictiveController(MPCParameters mpcParameters, double mass, double gravityZ, double nominalCoMHeight, double dt, YoRegistry parentRegistry)
   {
      this(new LinearMPCIndexHandler(numberOfBasisVectorsPerContactPoint), mpcParameters, mass, gravityZ, nominalCoMHeight, dt, parentRegistry);
   }

   public CoMTrajectoryModelPredictiveController(LinearMPCIndexHandler indexHandler,
                                                 MPCParameters mpcParameters,
                                                 double mass,
                                                 double gravityZ,
                                                 double nominalCoMHeight,
                                                 double dt,
                                                 YoRegistry parentRegistry)
   {
      super(indexHandler, mpcParameters, mass, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      firstVariableIndex = indexHandler::getComCoefficientStartIndex;

      qpSolver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   protected void initializeIndexHandler()
   {
      List<PreviewWindowSegment> planningWindow = previewWindowCalculator.getPlanningWindow();
      indexHandler.initialize(planningWindow);
   }

   @Override
   protected void resetActiveSet()
   {
      qpSolver.notifyResetActiveSet();
      qpSolver.resetRateRegularization();
   }

   @Override
   protected NativeMatrix solveQP()
   {
      qpSolver.initialize();
      qpSolver.submitMPCCommandList(mpcCommands);

      qpSolver.setUseWarmStart(useWarmStart.getBooleanValue());
      if (useWarmStart.getBooleanValue())
      {
         assembleActiveSet(firstVariableIndex);
         qpSolver.setPreviousSolution(previousSolution);
         qpSolver.setActiveInequalityIndices(activeInequalityConstraints);
      }

      if (!qpSolver.solve())
      {
         LogTools.info("Failed to find solution");
         extractNewActiveSetData(false, qpSolver, firstVariableIndex);

         return null;
      }

      NativeMatrix solutionCoefficients = qpSolver.getSolution();

      extractNewActiveSetData(true, qpSolver, firstVariableIndex);

      return solutionCoefficients;
   }


}
