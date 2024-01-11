package us.ihmc.rdx.simulation.scs2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Function;

public class RDXSCS2HumanoidSimulationManager extends RDXSCS2RestartableSimulationSession
{
   private final RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private final HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters;
   private final boolean useVelocityAndHeadingScript;
   private final DRCRobotModel robotModel;
   private final CommunicationMode ros2CommunicationMode;
   private final ArrayList<Function<ReferenceFrame, Robot>> secondaryRobots = new ArrayList<>();
   private final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private SCS2AvatarSimulation avatarSimulation;
   private Consumer<SCS2AvatarSimulationFactory> externalFactorySetup = null;

   public RDXSCS2HumanoidSimulationManager(RDXBaseUI baseUI, DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode)
   {
      this(baseUI, robotModel, ros2CommunicationMode, 0.3, 0.0, 0.0);
   }

   public RDXSCS2HumanoidSimulationManager(RDXBaseUI baseUI,
                                           DRCRobotModel robotModel,
                                           CommunicationMode ros2CommunicationMode,
                                           double initialYaw,
                                           double initialX,
                                           double initialY)
   {
      super(baseUI);

      this.robotModel = robotModel;
      this.ros2CommunicationMode = ros2CommunicationMode;

      setSessionBuilder(this::buildSession);
      getOnSessionStartedRunnables().add(() ->
      {
         avatarSimulation.beforeSessionThreadStart();
         avatarSimulation.afterSessionThreadStart();
      });
      getDestroyables().add(() ->
      {
         avatarSimulation.destroy();
      });

      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw, initialX, initialY);

      useVelocityAndHeadingScript = true;
      walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
   }

   public SimulationSession buildSession()
   {
      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(ros2CommunicationMode.getPubSubImplementation(), "humanoid_simulation");

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
      {
         avatarSimulationFactory.addTerrainObjectDefinition(terrainObjectDefinition);
      }
      for (Function<ReferenceFrame, Robot> secondaryRobot : secondaryRobots)
      {
         // FIXME Technically the inertial frame could be different here
         avatarSimulationFactory.addSecondaryRobot(secondaryRobot.apply(SimulationSession.DEFAULT_INERTIAL_FRAME));
      }
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setCreateYoVariableServer(true);
      avatarSimulationFactory.setUseBulletPhysicsEngine(true);
      avatarSimulationFactory.setUseRobotDefinitionCollisions(false);
      avatarSimulationFactory.setShowGUI(false);
      if (externalFactorySetup != null)
         externalFactorySetup.accept(avatarSimulationFactory);

      avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
      avatarSimulation.setSystemExitOnDestroy(false);

      return avatarSimulation.getSimulationConstructionSet().getSimulationSession();
   }

   public ArrayList<Function<ReferenceFrame, Robot>> getSecondaryRobots()
   {
      return secondaryRobots;
   }

   public ArrayList<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   public void setExternalFactorySetup(Consumer<SCS2AvatarSimulationFactory> externalFactorySetup)
   {
      this.externalFactorySetup = externalFactorySetup;
   }

   public SCS2AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }
}
