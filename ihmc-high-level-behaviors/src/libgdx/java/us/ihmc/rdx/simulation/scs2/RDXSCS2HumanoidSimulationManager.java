package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class RDXSCS2HumanoidSimulationManager extends RDXSCS2RestartableSimulationSession
{
   private final RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private final HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters;
   private final boolean useVelocityAndHeadingScript;
   private final DRCRobotModel robotModel;
   private final List<RobotDefinition> secondaryRobotDefinitions = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private SCS2AvatarSimulation avatarSimulation;
   private Consumer<SCS2AvatarSimulationFactory> externalFactorySetup = null;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXSCS2HumanoidSimulationManager(RDXBaseUI baseUI, DRCRobotModel robotModel)
   {
      this(baseUI, robotModel, 0.3, 0.0, 0.0);
   }

   public RDXSCS2HumanoidSimulationManager(RDXBaseUI baseUI,
                                           DRCRobotModel robotModel,
                                           double initialYaw,
                                           double initialX,
                                           double initialY)
   {
      super(baseUI);

      this.robotModel = robotModel;

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
      AsyncROS2Node asyncROS2Node = new AsyncROS2Node("humanoid_simulation");

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setAsyncROS2Node(asyncROS2Node);
      avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
      {
         avatarSimulationFactory.addTerrainObjectDefinition(terrainObjectDefinition);
      }
      for (RobotDefinition secondaryRobotDefinition : secondaryRobotDefinitions)
      {
         // FIXME Technically the inertial frame could be different here
         avatarSimulationFactory.addSecondaryRobot(new Robot(secondaryRobotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME));
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

      getAdditionalImGuiWidgets().add(() ->
      {
         if (ImGui.button(labels.get("Reinitialize State Estimator")))
         {
            avatarSimulation.reinitializeStateEstimatorFromRobot();
         }
         if (ImGui.button(labels.get("Reinitialize State Estimator to World Origin")))
         {
            avatarSimulation.reinitializeStateEstimatorToWorldOrigin();
         }
      });

      return avatarSimulation.getSimulationConstructionSet().getSimulationSession();
   }

   public List<RobotDefinition> getSecondaryRobotDefinitions()
   {
      return secondaryRobotDefinitions;
   }

   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
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
