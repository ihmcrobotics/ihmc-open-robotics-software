package us.ihmc.atlas;

import java.util.HashMap;
import java.util.Map;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasKinematicsStreamingToolboxModule extends KinematicsStreamingToolboxModule
{
   public AtlasKinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel, startYoVariableServer, pubSubImplementation);
      controller.setInitialRobotConfigurationNamedMap(initialConfiguration(robotModel));
      controller.getTools().getIKController().getCenterOfMassSafeMargin().set(0.10);
   }

   private static Map<String, Double> initialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      HumanoidJointNameMap jointMap = robotModel.getJointMap();

      initialConfigurationMap.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      initialConfigurationMap.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      initialConfigurationMap.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.5);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.0);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.5);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);

         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.3));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 2.0);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(0.5));
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), 0.0);
      }

      return initialConfigurationMap;
   }

   @Override
   public DataServerSettings getYoVariableServerSettings()
   {
      return new DataServerSettings(true);
   }

   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotModelFlag = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true)
                                                                    .setStringParser(JSAP.STRING_PARSER);

      robotModelFlag.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModelFlag);

      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      String robotVersionString = config.getString("robotModel");
      if (robotVersionString == null)
      {
         robotVersionString = "ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ";
      }
      LogTools.info("Using robot version: {}", robotVersionString);
      try
      {
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(robotVersionString, RobotTarget.SCS, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + robotVersionString);
         System.out.println(jsap.getHelp());

         return;
      }

      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      LogTools.info("Using ROS 2 {} mode.", pubSubImplementation.name());
      new AtlasKinematicsStreamingToolboxModule(robotModel, startYoVariableServer, pubSubImplementation);
   }
}
