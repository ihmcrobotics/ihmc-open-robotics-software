package us.ihmc.gdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.StatelessNotification;

import java.util.ArrayList;
import java.util.function.Consumer;

public class GDXSCS2EnvironmentManager
{
   private GDXSCS2SimulationSession scs2SimulationSession;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiPanel managerPanel = new ImGuiPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
   private SCS2AvatarSimulation avatarSimulation;
   private RealtimeROS2Node realtimeROS2Node;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private boolean useVelocityAndHeadingScript;
   private GDXImGuiBasedUI baseUI;
   private int recordFrequency;
   private DRCRobotModel robotModel;
   private CommunicationMode ros2CommunicationMode;
   private final ArrayList<GDXSCS2SecondaryRobot> secondaryRobots = new ArrayList<>();
   private final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final ArrayList<String> robotsToHide = new ArrayList<>();
   private volatile boolean starting = false;
   private volatile boolean started = false;
   private ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();
   private final StatelessNotification destroyedNotification = new StatelessNotification();
   private Consumer<SCS2AvatarSimulationFactory> externalFactorySetup = null;
   private boolean createYoVariableServer = true;

   public void create(GDXImGuiBasedUI baseUI, DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode)
   {
      this.baseUI = baseUI;
      this.robotModel = robotModel;
      this.ros2CommunicationMode = ros2CommunicationMode;

      double initialYaw = 0.3;
      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      //      recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));
      recordFrequency = 1;

      baseUI.getImGuiPanelManager().addPanel(managerPanel);
   }

   private void renderImGuiWidgets()
   {
      if (!starting && !started)
      {
         if (ImGui.button(labels.get("Build simulation")))
         {
            buildSimulation();
         }
      }
      if (started)
      {
         if (ImGui.button(labels.get("Rebuild simulation")))
         {
            destroy();
            buildSimulation(true);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Destroy")))
         {
            destroy();
         }
      }
      if (starting)
      {
         ImGui.text("Starting...");
      }

      if (started)
      {
         scs2SimulationSession.renderImGuiWidgets();
      }
   }

   public void buildSimulation()
   {
      buildSimulation(false);
   }

   public void buildSimulation(boolean waitForDestroy)
   {
      starting = true;
      ThreadTools.startAsDaemon(() ->
      {
         if (waitForDestroy)
            destroyedNotification.blockingWait();

         realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(ros2CommunicationMode.getPubSubImplementation(),
                                                             "flat_ground_walking_track_simulation");

         SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
         avatarSimulationFactory.setRobotModel(robotModel);
         avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
         avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(false, null);
         for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
         {
            avatarSimulationFactory.addTerrainObjectDefinition(terrainObjectDefinition);
         }
         for (GDXSCS2SecondaryRobot secondaryRobot : secondaryRobots)
         {
            avatarSimulationFactory.addSecondaryRobot(secondaryRobot.create());
         }
         avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
         avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
         avatarSimulationFactory.setCreateYoVariableServer(isCreateYoVariableServer());
         avatarSimulationFactory.setUseBulletPhysicsEngine(true);
         avatarSimulationFactory.setUseRobotDefinitionCollisions(true);
         avatarSimulationFactory.setShowGUI(false);
         if (externalFactorySetup != null)
            externalFactorySetup.accept(avatarSimulationFactory);

         avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
         avatarSimulation.setSystemExitOnDestroy(false);

         scs2SimulationSession = new GDXSCS2SimulationSession(avatarSimulation.getSimulationSession());
         scs2SimulationSession.getOnSessionStartedRunnables().addAll(onSessionStartedRunnables);

         avatarSimulation.beforeSessionThreadStart();

         scs2SimulationSession.create(baseUI, managerPanel);

         for (String robotToHide : robotsToHide)
         {
            scs2SimulationSession.getShowRobotMap().get(robotToHide).set(false);
         }

         avatarSimulation.afterSessionThreadStart();

         scs2SimulationSession.getControlPanel().getIsShowing().set(true);
         starting = false;
         started = true;
      }, getClass().getSimpleName() + "Build");
   }

   public void update()
   {
      if (started)
      {
         scs2SimulationSession.update();
      }
   }

   public void destroy()
   {
      if (started)
      {
         started = false;
         ThreadTools.startAsDaemon(() ->
         {
            avatarSimulation.destroy();
            scs2SimulationSession.destroy(baseUI);
            avatarSimulation = null;
            scs2SimulationSession = null;
            realtimeROS2Node = null;
            destroyedNotification.notifyOtherThread();
         }, getClass().getSimpleName() + "Destroy");
      }
   }

   public ArrayList<GDXSCS2SecondaryRobot> getSecondaryRobots()
   {
      return secondaryRobots;
   }

   public ArrayList<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   public GDXSCS2SimulationSession getSCS2SimulationSession()
   {
      return scs2SimulationSession;
   }

   public ArrayList<String> getRobotsToHide()
   {
      return robotsToHide;
   }

   public ArrayList<Runnable> getOnSessionStartedRunnables()
   {
      return onSessionStartedRunnables;
   }

   public void setExternalFactorySetup(Consumer<SCS2AvatarSimulationFactory> externalFactorySetup)
   {
      this.externalFactorySetup = externalFactorySetup;
   }

   public boolean isCreateYoVariableServer()
   {
      return createYoVariableServer;
   }

   public void setCreateYoVariableServer(boolean createYoVariableServer)
   {
      this.createYoVariableServer = createYoVariableServer;
   }
}
