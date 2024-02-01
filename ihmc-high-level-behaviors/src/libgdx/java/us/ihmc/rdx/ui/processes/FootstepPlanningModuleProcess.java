package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.pubsub.DomainFactory;

import java.util.function.Supplier;

public class FootstepPlanningModuleProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier;
   private FootstepPlanningModule footstepPlanningModule;

   public FootstepPlanningModuleProcess(Supplier<DRCRobotModel> robotModelSupplier, Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.pubSubImplementationSupplier = pubSubImplementationSupplier;
   }

   @Override
   protected void startInternal()
   {
      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(robotModelSupplier.get(), pubSubImplementationSupplier.get());
   }

   @Override
   protected void stopInternal()
   {
      footstepPlanningModule.closeAndDispose();
      footstepPlanningModule = null;
   }

   @Override
   public String getName()
   {
      return "Footstep planning module";
   }
}
