package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * This provider class is designed to provide a garbage-free way of generating all the desired input commands for the MPC core.
 */
public class CommandProvider
{
   private final RecyclingArrayList<CoMPositionCommand> comPositionCommandPool = new RecyclingArrayList<>(CoMPositionCommand::new);
   private final RecyclingArrayList<CoMVelocityCommand> comVelocityCommandPool = new RecyclingArrayList<>(CoMVelocityCommand::new);
   private final RecyclingArrayList<DCMPositionCommand> dcmPositionCommandPool = new RecyclingArrayList<>(DCMPositionCommand::new);
   private final RecyclingArrayList<DCMVelocityCommand> dcmVelocityCommandPool = new RecyclingArrayList<>(DCMVelocityCommand::new);
   private final RecyclingArrayList<VRPPositionCommand> vrpPositionCommandPool = new RecyclingArrayList<>(VRPPositionCommand::new);
   private final RecyclingArrayList<VRPVelocityCommand> vrpVelocityCommandPool = new RecyclingArrayList<>(VRPVelocityCommand::new);
   private final RecyclingArrayList<VRPTrackingCommand> vrpTrackingCommandPool = new RecyclingArrayList<>(VRPTrackingCommand::new);
   private final RecyclingArrayList<CoMPositionContinuityCommand> comPositionContinuityCommandPool = new RecyclingArrayList<>(CoMPositionContinuityCommand::new);
   private final RecyclingArrayList<CoMVelocityContinuityCommand> comVelocityContinuityCommandPool = new RecyclingArrayList<>(CoMVelocityContinuityCommand::new);
   private final RecyclingArrayList<VRPPositionContinuityCommand> vrpPositionContinuityCommandPool = new RecyclingArrayList<>(VRPPositionContinuityCommand::new);
   private final RecyclingArrayList<RhoAccelerationObjectiveCommand> rhoAccelerationObjectiveCommandPool = new RecyclingArrayList<>(RhoAccelerationObjectiveCommand::new);
   private final RecyclingArrayList<DiscreteAngularVelocityOrientationCommand> discreteOrientationCommandPool = new RecyclingArrayList<>(
         DiscreteAngularVelocityOrientationCommand::new);


   /**
          * Clears all the commands, resetting the provider. Must be called every iteration of the MPC
          */
   public void reset()
   {
      comPositionCommandPool.clear();
      comVelocityCommandPool.clear();
      dcmPositionCommandPool.clear();
      dcmVelocityCommandPool.clear();
      vrpPositionCommandPool.clear();
      vrpVelocityCommandPool.clear();
      vrpTrackingCommandPool.clear();
      comPositionContinuityCommandPool.clear();
      comVelocityContinuityCommandPool.clear();
      vrpPositionContinuityCommandPool.clear();
      rhoAccelerationObjectiveCommandPool.clear();
      discreteOrientationCommandPool.clear();
   }

   public CoMPositionCommand getNextCoMPositionCommand()
   {
      return comPositionCommandPool.add();
   }

   public CoMVelocityCommand getNextCoMVelocityCommand()
   {
      return comVelocityCommandPool.add();
   }

   public DCMPositionCommand getNextDCMPositionCommand()
   {
      return dcmPositionCommandPool.add();
   }

   public DCMVelocityCommand getNextDCMVelocityCommand()
   {
      return dcmVelocityCommandPool.add();
   }

   public VRPPositionCommand getNextVRPPositionCommand()
   {
      return vrpPositionCommandPool.add();
   }

   public VRPVelocityCommand getNextVRPVelocityCommand()
   {
      return vrpVelocityCommandPool.add();
   }

   public VRPTrackingCommand getNextVRPTrackingCommand()
   {
      return vrpTrackingCommandPool.add();
   }

   public CoMPositionContinuityCommand getNextComPositionContinuityCommand()
   {
      return comPositionContinuityCommandPool.add();
   }

   public CoMVelocityContinuityCommand getNextComVelocityContinuityCommand()
   {
      return comVelocityContinuityCommandPool.add();
   }

   public VRPPositionContinuityCommand getNextVRPPositionContinuityCommand()
   {
      return vrpPositionContinuityCommandPool.add();
   }

   public RhoAccelerationObjectiveCommand getNextRhoAccelerationObjectiveCommand()
   {
      return rhoAccelerationObjectiveCommandPool.add();
   }

   public DiscreteAngularVelocityOrientationCommand getNextDiscreteAngularVelocityOrientationCommand()
   {
      return discreteOrientationCommandPool.add();
   }
}
