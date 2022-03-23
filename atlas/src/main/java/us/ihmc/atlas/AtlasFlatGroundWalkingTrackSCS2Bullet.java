package us.ihmc.atlas;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasFlatGroundWalkingTrackSCS2Bullet
{
   private static final boolean USE_STAND_PREP = false;
   private static boolean createYoVariableServer = System.getProperty("create.yovariable.server") != null
         && Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private final RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS,
                                                                                      "flat_ground_walking_track_simulation");
   //private static final double SIMULATION_DT = 5e-4;

   public AtlasFlatGroundWalkingTrackSCS2Bullet()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      robotModel.setUseSDFCollisions(true);
      setCollisionGroupsMasks(robotModel.getRobotDefinition());
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      int recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));

      boolean useVelocityAndHeadingScript = true;
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();

      double initialYaw = 0.3;
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      if (USE_STAND_PREP)
         setupStandPrepController(robotModel, realtimeROS2Node, avatarSimulationFactory);
      else
         avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);
      avatarSimulationFactory.setUseBulletPhysicsEngine(true);
      avatarSimulationFactory.setUseDescriptionCollisions(true);
      
      //avatarSimulationFactory.setSimulationDT(SIMULATION_DT);

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      avatarSimulation.start();

      avatarSimulation.getSessionVisualizerControls().setCameraFocusPosition(0.3, 0.0, 1.0);
      avatarSimulation.getSessionVisualizerControls().setCameraPosition(7.0, 4.0, 3.0);
   }


   public static HighLevelHumanoidControllerFactory setupStandPrepController(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node, SCS2AvatarSimulationFactory simulationFactory)
   {
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      PushRecoveryControllerParameters pushRecoveryControllerParameters = robotModel.getPushRecoveryControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    feetContactSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    pushRecoveryControllerParameters,
                                                                                                    copTrajectoryParameters);
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.useDefaultStandPrepControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      controllerFactory.setInitialState(HighLevelControllerName.STAND_PREP_STATE);

      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node);
      simulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      return controllerFactory;
   }

   private void setCollisionGroupsMasks(RobotDefinition robotDefinition)
   {
      String defaultFilter = "DefaultFilter";
      String staticFilter = "StaticFilter";
      String kinematicFilter = "KinematicFilter";
      String debrisFilter = "DebrisFilter";
      String sensorTrigger = "SensorTrigger";
      String characterFilter = "CharacterFilter";
      String bodyName = "Body"; 
      String pelvis = "Pelvis";
      String rightLeg = "RightLeg";
      String leftLeg = "LeftLeg";
      String rightArm = "RightArm";
      String leftArm = "LeftArm";
      long bulletCollisionGroup;
      long bulletCollideMask;

      CollidableHelper helper = new CollidableHelper();

      // Set default Bullet collision groups/masks
      bulletCollisionGroup = helper.getCollisionMask(defaultFilter);
      bulletCollisionGroup = helper.getCollisionMask(staticFilter);
      bulletCollisionGroup = helper.getCollisionMask(kinematicFilter);
      bulletCollisionGroup = helper.getCollisionMask(debrisFilter);
      bulletCollisionGroup = helper.getCollisionMask(sensorTrigger);
      bulletCollisionGroup = helper.getCollisionMask(characterFilter);
      
      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (CollisionShapeDefinition shapeDefinition : rigidBodyDefinition.getCollisionShapeDefinitions())
         {
            if (shapeDefinition.getName().contains("pelvis")
                  || shapeDefinition.getName().contains("uglut")
                  || shapeDefinition.getName().contains("lglut")
                  )
            {
               bulletCollisionGroup = helper.getCollisionMask(pelvis);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, leftArm, rightArm);

            }
            else if (shapeDefinition.getName().contains("utorso")
                  || shapeDefinition.getName().contains("hokuyo") 
                  || shapeDefinition.getName().contains("head"))
            {
               bulletCollisionGroup = helper.getCollisionMask(bodyName);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, rightLeg, leftLeg, rightArm, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uleg")
                  || shapeDefinition.getName().contains("l_lleg")
                  || shapeDefinition.getName().contains("l_talus")
                  || shapeDefinition.getName().contains("l_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, rightLeg, leftArm);

            }
            else if (shapeDefinition.getName().contains("r_uleg")
                  || shapeDefinition.getName().contains("r_lleg")
                  || shapeDefinition.getName().contains("r_talus")
                  || shapeDefinition.getName().contains("r_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, leftLeg, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uarm")
                  || shapeDefinition.getName().contains("l_clav")
                  || shapeDefinition.getName().contains("l_scap")
                  || shapeDefinition.getName().contains("l_larm")
                  || shapeDefinition.getName().contains("l_ufarm")
                  || shapeDefinition.getName().contains("l_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, pelvis, leftLeg, rightLeg, rightArm);
            }
            else if (shapeDefinition.getName().contains("r_uarm")
                  || shapeDefinition.getName().contains("r_clav")
                  || shapeDefinition.getName().contains("r_scap")
                  || shapeDefinition.getName().contains("r_larm")
                  || shapeDefinition.getName().contains("r_ufarm")
                  || shapeDefinition.getName().contains("r_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, bodyName, pelvis, leftLeg, rightLeg, leftArm);
            }
            else
            {
               bulletCollisionGroup = 1;
               bulletCollideMask = 1 + 2;
            }

            shapeDefinition.setCollisionGroup(bulletCollisionGroup);
            shapeDefinition.setCollisionMask(bulletCollideMask);
         }
      }
   }

   public static void main(String[] args)
   {
      new AtlasFlatGroundWalkingTrackSCS2Bullet();
   }
}
