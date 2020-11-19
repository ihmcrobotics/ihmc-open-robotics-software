package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commons.lists.RecyclingArrayList;

public class CommandProvider
{
   private final RecyclingArrayList<CoMPositionCommand> comPositionCommandPool = new RecyclingArrayList<>(CoMPositionCommand::new);
   private final RecyclingArrayList<CoMVelocityCommand> comVelocityCommandPool = new RecyclingArrayList<>(CoMVelocityCommand::new);
   private final RecyclingArrayList<DCMPositionCommand> dcmPositionCommandPool = new RecyclingArrayList<>(DCMPositionCommand::new);
   private final RecyclingArrayList<DCMVelocityCommand> dcmVelocityCommandPool = new RecyclingArrayList<>(DCMVelocityCommand::new);
   private final RecyclingArrayList<VRPPositionCommand> vrpPositionCommandPool = new RecyclingArrayList<>(VRPPositionCommand::new);
   private final RecyclingArrayList<VRPVelocityCommand> vrpVelocityCommandPool = new RecyclingArrayList<>(VRPVelocityCommand::new);
   private final RecyclingArrayList<CoMPositionContinuityCommand> comPositionContinuityCommandPool = new RecyclingArrayList<>(CoMPositionContinuityCommand::new);
   private final RecyclingArrayList<CoMVelocityContinuityCommand> comVelocityContinuityCommandPool = new RecyclingArrayList<>(CoMVelocityContinuityCommand::new);
   private final RecyclingArrayList<RhoValueObjectiveCommand> rhoValueObjectiveCommandPool = new RecyclingArrayList<>(RhoValueObjectiveCommand::new);

   public void reset()
   {
      comPositionCommandPool.clear();
      comVelocityCommandPool.clear();
      dcmPositionCommandPool.clear();
      dcmVelocityCommandPool.clear();
      vrpPositionCommandPool.clear();
      vrpVelocityCommandPool.clear();
      comPositionContinuityCommandPool.clear();
      comVelocityContinuityCommandPool.clear();
      rhoValueObjectiveCommandPool.clear();
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

   public CoMPositionContinuityCommand getNextComPositionContinuityCommand()
   {
      return comPositionContinuityCommandPool.add();
   }

   public CoMVelocityContinuityCommand getNextComVelocityContinuityCommand()
   {
      return comVelocityContinuityCommandPool.add();
   }

   public RhoValueObjectiveCommand getNextRhoValueObjectiveCommand()
   {
      return rhoValueObjectiveCommandPool.add();
   }
}
