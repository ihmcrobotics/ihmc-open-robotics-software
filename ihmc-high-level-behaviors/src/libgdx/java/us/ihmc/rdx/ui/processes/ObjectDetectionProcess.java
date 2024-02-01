package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.pubsub.DomainFactory;

import java.util.function.Supplier;

public class ObjectDetectionProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier;
   private final Supplier<RobotTarget> robotTargetSupplier;
   private ObjectDetectorToolboxModule objectDetectorToolboxModule;
   private FiducialDetectorToolboxModule fiducialDetectorToolboxModule;

   public ObjectDetectionProcess(Supplier<DRCRobotModel> robotModelSupplier,
                                 Supplier<DomainFactory.PubSubImplementation> pubSubImplementationSupplier,
                                 Supplier<RobotTarget> robotTargetSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.pubSubImplementationSupplier = pubSubImplementationSupplier;
      this.robotTargetSupplier = robotTargetSupplier;
   }

   @Override
   protected void startInternal()
   {
      DRCRobotModel robotModel = robotModelSupplier.get();
      objectDetectorToolboxModule = new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                    robotModel.createFullRobotModel(),
                                                                    robotModel.getLogModelProvider(),
                                                                    pubSubImplementationSupplier.get());
      fiducialDetectorToolboxModule = new FiducialDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                        robotTargetSupplier.get(),
                                                                        robotModel.createFullRobotModel(),
                                                                        robotModel.getLogModelProvider(),
                                                                        pubSubImplementationSupplier.get());
   }

   @Override
   protected void stopInternal()
   {
      objectDetectorToolboxModule.destroy();
      fiducialDetectorToolboxModule.destroy();
   }

   @Override
   public String getName()
   {
      return "Object Detection";
   }
}
