package us.ihmc.valkyrie;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ValkyrieKinematicsStreamingToolboxModule extends KinematicsStreamingToolboxModule
{
   public ValkyrieKinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel, startYoVariableServer, pubSubImplementation);
      controller.setInitialRobotConfigurationNamedMap(createInitialConfiguration(robotModel));
   }

   @Override
   public DataServerSettings getYoVariableServerSettings()
   {
      return new DataServerSettings(true);
   }

   private static Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      defaultRobotInitialSetup.initializeRobot(robot, jointMap);

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();
         double q_priv = robot.getOneDegreeOfFreedomJoint(jointName).getQ();
         initialConfigurationMap.put(jointName, q_priv);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.6);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.5);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.0);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.5);
      }

      return initialConfigurationMap;
   }

   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      ValkyrieKinematicsStreamingToolboxModule module = new ValkyrieKinematicsStreamingToolboxModule(robotModel, startYoVariableServer, pubSubImplementation);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         System.out.println("Shutting down " + ValkyrieKinematicsStreamingToolboxModule.class.getSimpleName());
         module.closeAndDispose();
      }));
   }
}
