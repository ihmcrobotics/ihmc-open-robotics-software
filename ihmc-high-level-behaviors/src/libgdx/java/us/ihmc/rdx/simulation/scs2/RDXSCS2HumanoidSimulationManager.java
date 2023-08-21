package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.StatelessNotification;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Function;

public class RDXSCS2HumanoidSimulationManager
{
   private RDXSCS2SimulationSession scs2SimulationSession;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel managerPanel = new RDXPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
   private SCS2AvatarSimulation avatarSimulation;
   private RealtimeROS2Node realtimeROS2Node;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters;
   private boolean useVelocityAndHeadingScript;
   private RDXBaseUI baseUI;
   private int recordFrequency;
   private DRCRobotModel robotModel;
   private CommunicationMode ros2CommunicationMode;
   private final ArrayList<Function<ReferenceFrame, Robot>> secondaryRobots = new ArrayList<>();
   private final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final ArrayList<String> robotsToHide = new ArrayList<>();
   private volatile boolean starting = false;
   private volatile boolean started = false;
   private final ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();
   private final StatelessNotification destroyedNotification = new StatelessNotification();
   private Consumer<SCS2AvatarSimulationFactory> externalFactorySetup = null;

   public void create(RDXBaseUI baseUI, DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode)
   {
      create(baseUI, robotModel, ros2CommunicationMode, 0.3, 0.0, 0.0);
   }

   public void create(RDXBaseUI baseUI, DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode, double initialYaw, double initialX, double initialY)
   {
      this.baseUI = baseUI;
      this.robotModel = robotModel;
      this.ros2CommunicationMode = ros2CommunicationMode;

      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw, initialX, initialY);

      //      recordFrequency = (int) Math.max(1.0, Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));
      recordFrequency = 1;

      useVelocityAndHeadingScript = true;
      walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();

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
         avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, walkingScriptParameters);
         for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
         {
            avatarSimulationFactory.addTerrainObjectDefinition(terrainObjectDefinition);
         }
         for (Function<ReferenceFrame, Robot> secondaryRobot : secondaryRobots)
         {
            avatarSimulationFactory.addSecondaryRobot(secondaryRobot.apply(scs2SimulationSession.getSession().getInertialFrame()));
         }
         avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
         avatarSimulationFactory.setSimulationDataRecordTickPeriod(recordFrequency);
         avatarSimulationFactory.setCreateYoVariableServer(true);
         avatarSimulationFactory.setUseBulletPhysicsEngine(true);
         avatarSimulationFactory.setUseRobotDefinitionCollisions(true);
         avatarSimulationFactory.setShowGUI(false);
         if (externalFactorySetup != null)
            externalFactorySetup.accept(avatarSimulationFactory);

         avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
         avatarSimulation.setSystemExitOnDestroy(false);

         scs2SimulationSession = new RDXSCS2SimulationSession();
         scs2SimulationSession.create(baseUI, managerPanel);
         scs2SimulationSession.startSession(avatarSimulation.getSimulationConstructionSet().getSimulationSession());
         scs2SimulationSession.getOnSessionStartedRunnables().addAll(onSessionStartedRunnables);

         avatarSimulation.beforeSessionThreadStart();


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

   public ArrayList<Function<ReferenceFrame, Robot>> getSecondaryRobots()
   {
      return secondaryRobots;
   }

   public ArrayList<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   public RDXSCS2Session getSCS2SimulationSession()
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

   public SCS2AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }
}
