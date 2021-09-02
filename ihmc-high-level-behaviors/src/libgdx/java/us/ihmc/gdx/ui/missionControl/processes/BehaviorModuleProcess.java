package us.ihmc.gdx.ui.missionControl.processes;

import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.log.LogTools;

import java.util.function.Supplier;

public class BehaviorModuleProcess extends RestartableMissionControlProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final ImInt ros2Mode;
   private final ImInt messagerMode;
   // TODO: GUI selection
   private final GDXBehaviorUIRegistry behaviorRegistry;
   private BehaviorModule behaviorModule;

   public BehaviorModuleProcess(Supplier<DRCRobotModel> robotModelSupplier, ImInt ros2Mode, ImInt messagerMode, GDXBehaviorUIRegistry behaviorRegistry)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.ros2Mode = ros2Mode;
      this.messagerMode = messagerMode;
      this.behaviorRegistry = behaviorRegistry;
   }

   @Override
   protected void startInternal()
   {
      LogTools.info("Starting behavior module");
      behaviorModule = new BehaviorModule(behaviorRegistry,
                                          robotModelSupplier.get(),
                                          CommunicationMode.fromOrdinal(ros2Mode.get()),
                                          CommunicationMode.fromOrdinal(messagerMode.get()));
   }

   @Override
   protected void stopInternal()
   {
      behaviorModule.destroy();
      behaviorModule = null;
   }

   @Override
   public String getName()
   {
      return "Behavior module";
   }
}
