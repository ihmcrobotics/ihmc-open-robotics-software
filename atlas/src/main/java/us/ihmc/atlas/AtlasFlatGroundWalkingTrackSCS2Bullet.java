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
//   private static final double SIMULATION_DT = 0.001;

   public AtlasFlatGroundWalkingTrackSCS2Bullet()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      robotModel.setUseSDFCollisions(true);
      //setCollisionGroupsMasks(robotModel.getRobotDefinition());
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
      //Leave commented out - causes the feet to be jittery.
//      avatarSimulation.getSimulationSession().setSessionDTSeconds(robotModel.getEstimatorDT());

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
      
      //TODO: Add arms, hands, bottom parts of legs, and feet to the collision groups/masks.
      String bodyName = "Body"; 
      String pelvis = "Pelvis";
      String upperRightLeg = "UpperRightLeg";
      String upperLeftLeg = "UpperLeftLeg";
      long collisionMask;
      long collisionGroup;

      CollidableHelper helper = new CollidableHelper();

      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (CollisionShapeDefinition shapeDefinition : rigidBodyDefinition.getCollisionShapeDefinitions())
         {
            if (shapeDefinition.getName().equals("pelvis_collision")
                  || shapeDefinition.getName().equals("l_uglut_collision")
                  || shapeDefinition.getName().equals("r_uglut_collision")
                  || shapeDefinition.getName().equals("l_lglut_collision")
                  || shapeDefinition.getName().equals("r_lglut_collision")
                  || shapeDefinition.getName().equals("mtorso_collision"))
            {
               collisionMask = helper.getCollisionMask(pelvis);
               collisionGroup = helper.createCollisionGroup(bodyName, upperLeftLeg, upperRightLeg);
               System.out.println("pelvis " + collisionMask + " " + collisionGroup);
               shapeDefinition.setCollisionGroup(collisionGroup);
               shapeDefinition.setCollisionMask(collisionMask);
            }
            else if (shapeDefinition.getName().equals("utorso_collision") || shapeDefinition.getName().equals("utorso_collision")
                  || shapeDefinition.getName().equals("hokuyo_link_collision") || shapeDefinition.getName().equals("head_collision"))
            {
               collisionMask = helper.getCollisionMask(bodyName);
               collisionGroup = helper.createCollisionGroup(upperRightLeg, upperLeftLeg);
               System.out.println("bodyName " + collisionMask + " " + collisionGroup);
               shapeDefinition.setCollisionGroup(collisionGroup);
               shapeDefinition.setCollisionMask(collisionMask);
            }
            else if (shapeDefinition.getName().equals("l_uleg_collision"))
            {
               collisionMask = helper.getCollisionMask(upperLeftLeg);
               collisionGroup = helper.createCollisionGroup(bodyName, pelvis, upperRightLeg);
               System.out.println("left Leg " + collisionMask + " " + collisionGroup);
               shapeDefinition.setCollisionGroup(collisionGroup);
               shapeDefinition.setCollisionMask(collisionMask);
            }
            else if (shapeDefinition.getName().equals("r_uleg_collision"))
            {
               collisionMask = helper.getCollisionMask(upperRightLeg);
               collisionGroup = helper.createCollisionGroup(bodyName, pelvis, upperLeftLeg);
               System.out.println("right Leg " + collisionMask + " " + collisionGroup);
               shapeDefinition.setCollisionGroup(collisionGroup);
               shapeDefinition.setCollisionMask(collisionMask);
            }
         }
      }
   }

   public static void main(String[] args)
   {
      new AtlasFlatGroundWalkingTrackSCS2Bullet();
   }
}
