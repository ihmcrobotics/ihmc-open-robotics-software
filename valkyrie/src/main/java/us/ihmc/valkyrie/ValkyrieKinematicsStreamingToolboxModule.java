package us.ihmc.valkyrie;

import java.util.HashMap;
import java.util.Map;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieNetworkProcessor.NetworkProcessorVersion;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ValkyrieKinematicsStreamingToolboxModule extends KinematicsStreamingToolboxModule
{
   /**
    * The idea is to have a quick way to switch to a safer and more conservative operation mode meant
    * for a random random person to try out the IK streaming.
    */
   public static final boolean DEMO_MODE = false;

   public ValkyrieKinematicsStreamingToolboxModule(DRCRobotModel robotModel,
                                                   KinematicsStreamingToolboxParameters parameters,
                                                   boolean startYoVariableServer,
                                                   PubSubImplementation pubSubImplementation)
   {
      super(robotModel, parameters, startYoVariableServer, pubSubImplementation);
      controller.setInitialRobotConfigurationNamedMap(createInitialConfiguration(robotModel));
   }

   @Override
   public DataServerSettings createYoVariableServerSettings()
   {
      return super.createYoVariableServerSettings(true);
   }

   private static Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      defaultRobotInitialSetup.initializeRobot(robot);

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
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      if (DEMO_MODE)
      {
         ValkyrieKinematicsCollisionModel kinematicsCollisionModel = new ValkyrieKinematicsCollisionModel(jointMap);
         kinematicsCollisionModel.setEnableConservativeCollisions(true);
         robotModel.setHumanoidRobotKinematicsCollisionModel(kinematicsCollisionModel);
      }
      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;

      KinematicsStreamingToolboxParameters parameters = KinematicsStreamingToolboxParameters.defaultParameters();
      KinematicsStreamingToolboxConfigurationMessage defaultConfiguration = parameters.getDefaultConfiguration();

      if (NetworkProcessorVersion.fromEnvironment() == NetworkProcessorVersion.IHMC)
      {
         parameters.setCenterOfMassSafeMargin(0.05);
         if (robotModel.getRobotVersion() == ValkyrieRobotVersion.ARM_MASS_SIM)
         {
            defaultConfiguration.setEnableLeftHandTaskspace(false);
            defaultConfiguration.setEnableRightHandTaskspace(false);
            defaultConfiguration.setEnableLeftArmJointspace(true);
            defaultConfiguration.setEnableRightArmJointspace(true);
         }
      }

      if (DEMO_MODE)
      {
         // Add safety parameters here
         defaultConfiguration.setLockPelvis(true);
         defaultConfiguration.setEnablePelvisTaskspace(false);

         parameters.setDefaultAngularRateLimit(5.0);
         parameters.setInputPoseLPFBreakFrequency(2.0);
         parameters.setMinimizeAngularMomentum(true);
         parameters.setMinimizeLinearMomentum(true);
         parameters.setAngularMomentumWeight(0.25);
         parameters.setLinearMomentumWeight(0.25);

         // Restrict joint limits
         robotModel.setRobotDefinitionMutator(robotDefinition ->
         {
            OneDoFJointDefinition spineYaw = robotDefinition.getOneDoFJointDefinition(jointMap.getSpineJointName(SpineJointName.SPINE_YAW));
            spineYaw.setPositionLimits(Math.toRadians(-45.0), Math.toRadians(45.0));
            OneDoFJointDefinition spineRoll = robotDefinition.getOneDoFJointDefinition(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL));
            spineRoll.setPositionLimits(-0.1, 0.1);
            OneDoFJointDefinition spinePitch = robotDefinition.getOneDoFJointDefinition(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH));
            spinePitch.setPositionLimits(-0.1, 0.4);

            for (RobotSide robotSide : RobotSide.values)
            {
               OneDoFJointDefinition shoulderPitch = robotDefinition.getOneDoFJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH));
               shoulderPitch.setPositionLimits(-1.5, 0.8);

               OneDoFJointDefinition shoulderRoll = robotDefinition.getOneDoFJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL));
               if (robotSide == RobotSide.LEFT)
                  shoulderRoll.setPositionLimits(-1.3, 0.3);
               else
                  shoulderRoll.setPositionLimits(-0.3, 1.3);

               OneDoFJointDefinition shoulderYaw = robotDefinition.getOneDoFJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW));
               shoulderYaw.setPositionLimits(-1.5, 1.5);
            }
         });
      }

      ValkyrieKinematicsStreamingToolboxModule module = new ValkyrieKinematicsStreamingToolboxModule(robotModel,
                                                                                                     parameters,
                                                                                                     startYoVariableServer,
                                                                                                     pubSubImplementation);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         System.out.println("Shutting down " + ValkyrieKinematicsStreamingToolboxModule.class.getSimpleName());
         module.closeAndDispose();
      }));
   }
}
