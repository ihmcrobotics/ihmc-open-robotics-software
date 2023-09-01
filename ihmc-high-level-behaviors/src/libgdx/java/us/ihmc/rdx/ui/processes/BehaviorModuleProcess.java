package us.ihmc.rdx.ui.processes;

import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIRegistry;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.log.LogTools;

import java.util.function.Supplier;

public class BehaviorModuleProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final ImInt ros2Mode;
   // TODO: GUI selection
   private RDXBehaviorUIRegistry behaviorRegistry;
   private BehaviorModule behaviorModule;

   public BehaviorModuleProcess(Supplier<DRCRobotModel> robotModelSupplier, ImInt ros2Mode, RDXBehaviorUIRegistry behaviorRegistry)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.ros2Mode = ros2Mode;
      this.behaviorRegistry = behaviorRegistry;
   }

   @Override
   protected void startInternal()
   {
      LogTools.info("Starting behavior module");
      behaviorModule = new BehaviorModule(behaviorRegistry, robotModelSupplier.get(), CommunicationMode.fromOrdinal(ros2Mode.get()));
   }

   @Override
   protected void stopInternal()
   {
      behaviorModule.destroy();
      behaviorModule = null;
   }

   public void setBehaviorRegistry(RDXBehaviorUIRegistry behaviorRegistry)
   {
      this.behaviorRegistry = behaviorRegistry;
   }

   @Override
   public String getName()
   {
      return "Behavior module";
   }
}
