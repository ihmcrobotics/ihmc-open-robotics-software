package us.ihmc.atlas;

import com.martiansoftware.jsap.JSAPException;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasObjectDetectionModules
{

   private final ObjectDetectorToolboxModule objectDetectorToolboxModule;
   private final FiducialDetectorToolboxModule fiducialDetectorToolboxModule;

   public AtlasObjectDetectionModules()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      fiducialDetectorToolboxModule = new FiducialDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                        robotModel.getTarget(),
                                                                        robotModel.createFullRobotModel(),
                                                                        robotModel.getLogModelProvider(),
                                                                        PubSubImplementation.FAST_RTPS);
      objectDetectorToolboxModule = new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                    robotModel.createFullRobotModel(),
                                                                    robotModel.getLogModelProvider(),
                                                                    PubSubImplementation.FAST_RTPS);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         fiducialDetectorToolboxModule.closeAndDispose();
         objectDetectorToolboxModule.closeAndDispose();
         ThreadTools.sleep(10);
      }, getClass().getSimpleName() + "Shutdown"));
   }

   public static void main(String[] args) throws JSAPException
   {
      new AtlasObjectDetectionModules();
   }
}
