package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCSolutionInspection;

public class ContinuousMPCSolutionInspection extends LinearMPCSolutionInspection
{
   private static final double epsilon = 1e-3;
   private final ContinuousMPCIndexHandler indexHandler;
   private final ContinuousMPCQPInputCalculator inputCalculator;

   public ContinuousMPCSolutionInspection(ContinuousMPCIndexHandler indexHandler, double mass, double gravityZ)
   {
      super(indexHandler, gravityZ);
      this.indexHandler = indexHandler;
      inputCalculator = new ContinuousMPCQPInputCalculator(indexHandler, mass, gravityZ);
   }

   @Override
   public void inspectSolution(MPCCommandList commandList, DMatrixRMaj solution)
   {
      qpInputTypeA.setNumberOfVariables(indexHandler.getTotalProblemSize());
      qpInputTypeC.setNumberOfVariables(indexHandler.getTotalProblemSize());

      for (int i = 0; i < commandList.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = commandList.getCommand(i);

         switch (command.getCommandType())
         {
            case VALUE:
               inspectMPCValueObjective((MPCValueCommand) command, solution);
               break;
            case CONTINUITY:
               inspectCoMContinuityObjective((MPCContinuityCommand) command, solution);
               break;
            case LIST:
               inspectSolution((MPCCommandList) command, solution);
               break;
            case RHO_VALUE:
               inspectRhoValueCommand((RhoValueObjectiveCommand) command, solution);
               break;
            case VRP_TRACKING:
               inspectVRPTrackingObjective((VRPTrackingCommand) command, solution);
               break;
            case ORIENTATION_TRACKING:
               inspectOrientationTrackingObjective((OrientationTrackingCommand) command, solution);
               break;
            case ORIENTATION_DYNAMICS:
               // TODO
               break;
            default:
               throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   @Override
   public void inspectMPCValueObjective(MPCValueCommand command, DMatrixRMaj solution)
   {
      boolean success = inputCalculator.calculateValueObjective(qpInputTypeA, command);
      if (success)
         command.setCostToGo(inspectInput(qpInputTypeA, solution));
   }

   @Override
   public void inspectCoMContinuityObjective(MPCContinuityCommand command, DMatrixRMaj solution)
   {
      boolean success = inputCalculator.calculateCoMContinuityObjective(qpInputTypeA, command);
      if (success)
         inspectInput(qpInputTypeA, solution);
   }

   public void inspectOrientationTrackingObjective(OrientationTrackingCommand command, DMatrixRMaj solution)
   {
      boolean success = inputCalculator.calculateOrientationTrackingObjective(qpInputTypeC, command);
      if (success)
         command.setCostToGo(inspectInput(qpInputTypeC, solution));
   }
}
