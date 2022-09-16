package us.ihmc.valkyrie.simulation;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieKinematicsToolboxLauncher
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
      int updatePeriodMillis = 2;
      KinematicsToolboxModule module = new KinematicsToolboxModule(robotModel, true, updatePeriodMillis, false, DomainFactory.PubSubImplementation.FAST_RTPS);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         LogTools.info("Shutting down network processor modules.");
                                                         module.closeAndDispose();
                                                         ThreadTools.sleep(10);
                                                      }, ValkyrieKinematicsToolboxLauncher.class.getSimpleName() + "Shutdown"));
   }
}
