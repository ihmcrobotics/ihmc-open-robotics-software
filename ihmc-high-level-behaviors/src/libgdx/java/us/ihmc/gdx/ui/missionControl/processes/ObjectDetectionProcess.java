package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.pubsub.DomainFactory;

import java.util.function.Supplier;

public class ObjectDetectionProcess extends RestartableMissionControlProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier;
   private ObjectDetectorToolboxModule objectDetectorToolboxModule;

   public ObjectDetectionProcess(Supplier<DRCRobotModel> robotModelSupplier, Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.pubSubImplementationSupplier = pubSubImplementationSupplier;
   }

   @Override
   protected void startInternal()
   {
      DRCRobotModel robotModel = robotModelSupplier.get();
      objectDetectorToolboxModule = new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                    robotModel.createFullRobotModel(),
                                                                    robotModel.getLogModelProvider(),
                                                                    pubSubImplementationSupplier.get());
   }

   @Override
   protected void stopInternal()
   {
      objectDetectorToolboxModule.destroy();
   }

   @Override
   public String getName()
   {
      return "Object Detection";
   }
}
