package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.tools.thread.StatelessNotification;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

public class RDXSCS2RestartableSimulationSession
{
   private RDXSCS2SimulationSession scs2SimulationSession;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel managerPanel = new RDXPanel("SCS 2 Simulation Session", this::renderImGuiWidgets);
   private final RDXBaseUI baseUI;
   private final Supplier<SimulationSession> sessionSupplier;
   private final ArrayList<Function<ReferenceFrame, Robot>> secondaryRobots = new ArrayList<>();
   private final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final ArrayList<String> robotsToHide = new ArrayList<>();
   private final ArrayList<String> variableWidgets = new ArrayList<>();
   private volatile boolean starting = false;
   private volatile boolean started = false;
   private final ArrayList<Runnable> onSessionStartedRunnables = new ArrayList<>();
   private final StatelessNotification destroyedNotification = new StatelessNotification();

   public RDXSCS2RestartableSimulationSession(RDXBaseUI baseUI, Supplier<SimulationSession> sessionSupplier)
   {
      this.baseUI = baseUI;
      this.sessionSupplier = sessionSupplier;

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
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
      if (waitForDestroy)
         destroyedNotification.blockingWait();

      scs2SimulationSession = new RDXSCS2SimulationSession();
      scs2SimulationSession.getOnSessionStartedRunnables().addAll(onSessionStartedRunnables);
      scs2SimulationSession.create(baseUI, managerPanel);
      scs2SimulationSession.startSession(sessionSupplier.get());

      for (String yoVariableName : variableWidgets)
      {
         scs2SimulationSession.getPlotManager().addVariableWidget(yoVariableName);
      }

      for (String robotToHide : robotsToHide)
      {
         scs2SimulationSession.getShowRobotMap().get(robotToHide).set(false);
      }

      scs2SimulationSession.getControlPanel().getIsShowing().set(true);
      starting = false;
      started = true;
   }

   public void update()
   {
      if (started)
      {
         scs2SimulationSession.update();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (scs2SimulationSession != null)
      {
         scs2SimulationSession.getRenderables(renderables, pool, sceneLevels);
      }
   }

   public void destroy()
   {
      if (started)
      {
         started = false;
         ThreadTools.startAsDaemon(() ->
         {
            scs2SimulationSession.destroy(baseUI);
            scs2SimulationSession = null;
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

   public void addVariableWidget(String yoVariableName)
   {
      variableWidgets.add(yoVariableName);
   }
}
