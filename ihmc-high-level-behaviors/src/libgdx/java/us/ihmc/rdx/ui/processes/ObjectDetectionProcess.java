package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;

import java.util.function.Supplier;

public class ObjectDetectionProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private final Supplier<RobotTarget> robotTargetSupplier;
   private ObjectDetectorToolboxModule objectDetectorToolboxModule;
   private FiducialDetectorToolboxModule fiducialDetectorToolboxModule;

   public ObjectDetectionProcess(Supplier<DRCRobotModel> robotModelSupplier,
                                 Supplier<RobotTarget> robotTargetSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
      this.robotTargetSupplier = robotTargetSupplier;
   }

   @Override
   protected void startInternal()
   {
      DRCRobotModel robotModel = robotModelSupplier.get();
      objectDetectorToolboxModule = new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                    robotModel.createFullRobotModel(),
                                                                    robotModel.getLogModelProvider());
      fiducialDetectorToolboxModule = new FiducialDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                        robotTargetSupplier.get(),
                                                                        robotModel.createFullRobotModel(),
                                                                        robotModel.getLogModelProvider());
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
