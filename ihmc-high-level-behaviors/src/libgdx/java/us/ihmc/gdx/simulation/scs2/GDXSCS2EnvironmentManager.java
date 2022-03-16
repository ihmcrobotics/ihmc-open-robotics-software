package us.ihmc.gdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.simulation.environment.object.objects.FlatGroundDefinition;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public class GDXSCS2EnvironmentManager
{
   private GDXSCS2SimulationSession scs2SimulationSession;
   private final ImGuiPanel managerPanel = new ImGuiPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
   private SCS2AvatarSimulation avatarSimulation;
   private RealtimeROS2Node realtimeROS2Node;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters;
   private boolean useVelocityAndHeadingScript;
   private GDXImGuiBasedUI baseUI;
   private int recordFrequency;
   private DRCRobotModel robotModel;

   public void create(GDXImGuiBasedUI baseUI, DRCRobotModel robotModel)
   {
      this.baseUI = baseUI;
      this.robotModel = robotModel;

      recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));

      useVelocityAndHeadingScript = true;
      walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();

      double initialYaw = 0.3;
      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      rebuildSimulation();
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.button("Rebuild simulation"))
      {
         rebuildSimulation();
      }
      scs2SimulationSession.renderImGuiWidgets();
   }

   private void rebuildSimulation()
   {
      if (scs2SimulationSession != null)
      {
         baseUI.getImGuiPanelManager().queueRemovePanel(managerPanel);
         destroy(baseUI);
      }

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.INTRAPROCESS,
                                                          "flat_ground_walking_track_simulation");

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
      avatarSimulationFactory.setTerrainObjectDefinition(new FlatGroundDefinition());
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
      avatarSimulationFactory.setCreateYoVariableServer(false);
      avatarSimulationFactory.setUseBulletPhysicsEngine(true);
      avatarSimulationFactory.setUseDescriptionCollisions(true);
      avatarSimulationFactory.setShowGUI(false);

      avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
      avatarSimulation.setSystemExitOnDestroy(false);

      scs2SimulationSession = new GDXSCS2SimulationSession(avatarSimulation.getSimulationSession());

      avatarSimulation.beforeSessionThreadStart();

      scs2SimulationSession.setDT(robotModel.getEstimatorDT());
      scs2SimulationSession.create(baseUI, managerPanel);

      avatarSimulation.afterSessionThreadStart();

      scs2SimulationSession.getControlPanel().getIsShowing().set(true);
      baseUI.getImGuiPanelManager().queueAddPanel(managerPanel);
   }

   public void update()
   {
      scs2SimulationSession.update();
   }

   public void destroy(GDXImGuiBasedUI baseUI)
   {
      avatarSimulation.destroy();
      scs2SimulationSession.destroy(baseUI);
   }

   public GDXSCS2SimulationSession getSCS2SimulationSession()
   {
      return scs2SimulationSession;
   }
}
