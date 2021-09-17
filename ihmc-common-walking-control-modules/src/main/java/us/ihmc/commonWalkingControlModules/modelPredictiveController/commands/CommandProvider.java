package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * This provider class is designed to provide a garbage-free way of generating all the desired input commands for the MPC core.
 */
public class CommandProvider
{
   private final RecyclingArrayList<CoMPositionCommand> comPositionCommandPool = new RecyclingArrayList<>(CoMPositionCommand::new);
   private final RecyclingArrayList<CoMVelocityCommand> comVelocityCommandPool = new RecyclingArrayList<>(CoMVelocityCommand::new);
   private final RecyclingArrayList<CoMAccelerationCommand> comAccelerationCommandPool = new RecyclingArrayList<>(CoMAccelerationCommand::new);
   private final RecyclingArrayList<DCMPositionCommand> dcmPositionCommandPool = new RecyclingArrayList<>(DCMPositionCommand::new);
   private final RecyclingArrayList<DCMVelocityCommand> dcmVelocityCommandPool = new RecyclingArrayList<>(DCMVelocityCommand::new);
   private final RecyclingArrayList<VRPPositionCommand> vrpPositionCommandPool = new RecyclingArrayList<>(VRPPositionCommand::new);
   private final RecyclingArrayList<VRPVelocityCommand> vrpVelocityCommandPool = new RecyclingArrayList<>(VRPVelocityCommand::new);
   private final RecyclingArrayList<VRPTrackingCommand> vrpTrackingCommandPool = new RecyclingArrayList<>(VRPTrackingCommand::new);
   private final RecyclingArrayList<CoMPositionContinuityCommand> comPositionContinuityCommandPool = new RecyclingArrayList<>(CoMPositionContinuityCommand::new);
   private final RecyclingArrayList<CoMVelocityContinuityCommand> comVelocityContinuityCommandPool = new RecyclingArrayList<>(CoMVelocityContinuityCommand::new);
   private final RecyclingArrayList<VRPPositionContinuityCommand> vrpPositionContinuityCommandPool = new RecyclingArrayList<>(VRPPositionContinuityCommand::new);
   private final RecyclingArrayList<RhoBoundCommand> rhoBoundCommandPool = new RecyclingArrayList<>(RhoBoundCommand::new);
   private final RecyclingArrayList<NormalForceBoundCommand> normalForceBoundCommandPool = new RecyclingArrayList<>(NormalForceBoundCommand::new);
   private final RecyclingArrayList<OrientationValueCommand> orientationValueCommandPool = new RecyclingArrayList<>(OrientationValueCommand::new);
   private final RecyclingArrayList<DirectOrientationValueCommand> directOrientationValueCommandPool = new RecyclingArrayList<>(DirectOrientationValueCommand::new);
   private final RecyclingArrayList<OrientationContinuityCommand> orientationContinuityCommandPool = new RecyclingArrayList<>(OrientationContinuityCommand::new);
   private final RecyclingArrayList<ForceTrackingCommand> forceTrackingCommandPool = new RecyclingArrayList<>(ForceTrackingCommand::new);
   private final RecyclingArrayList<RhoTrackingCommand> rhoTrackingCommandPool = new RecyclingArrayList<>(RhoTrackingCommand::new);
   private final RecyclingArrayList<RhoRateTrackingCommand> rhoRateTrackingCommandPool = new RecyclingArrayList<>(RhoRateTrackingCommand::new);

   /**
          * Clears all the commands, resetting the provider. Must be called every iteration of the MPC
          */
   public void reset()
   {
      comPositionCommandPool.clear();
      comVelocityCommandPool.clear();
      comAccelerationCommandPool.clear();
      dcmPositionCommandPool.clear();
      dcmVelocityCommandPool.clear();
      vrpPositionCommandPool.clear();
      vrpVelocityCommandPool.clear();
      vrpTrackingCommandPool.clear();
      comPositionContinuityCommandPool.clear();
      comVelocityContinuityCommandPool.clear();
      vrpPositionContinuityCommandPool.clear();
      rhoBoundCommandPool.clear();
      normalForceBoundCommandPool.clear();
      orientationValueCommandPool.clear();
      directOrientationValueCommandPool.clear();
      orientationContinuityCommandPool.clear();
      forceTrackingCommandPool.clear();
      rhoTrackingCommandPool.clear();
      rhoRateTrackingCommandPool.clear();
   }

   public CoMPositionCommand getNextCoMPositionCommand()
   {
      return comPositionCommandPool.add();
   }

   public CoMVelocityCommand getNextCoMVelocityCommand()
   {
      return comVelocityCommandPool.add();
   }

   public CoMAccelerationCommand getNextCoMAccelerationCommand()
   {
      return comAccelerationCommandPool.add();
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

   public RhoBoundCommand getNextRhoBoundCommand()
   {
      return rhoBoundCommandPool.add();
   }

   public NormalForceBoundCommand getNextNormalForceBoundCommand()
   {
      return normalForceBoundCommandPool.add();
   }

   public DirectOrientationValueCommand getNextDirectOrientationValueCommand()
   {
      return directOrientationValueCommandPool.add();
   }

   public OrientationValueCommand getNextOrientationValueCommand()
   {
      return orientationValueCommandPool.add();
   }

   public OrientationContinuityCommand getNextOrientationContinuityCommand()
   {
      return orientationContinuityCommandPool.add();
   }

   public ForceTrackingCommand getForceTrackingCommand()
   {
      return forceTrackingCommandPool.add();
   }

   public RhoTrackingCommand getRhoMinimizationCommand()
   {
      return rhoTrackingCommandPool.add();
   }

   public RhoRateTrackingCommand getRhoRateMinimizationCommand()
   {
      return rhoRateTrackingCommandPool.add();
   }
}
