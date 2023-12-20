package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.tools.thread.StatelessNotification;

import java.util.ArrayList;
import java.util.function.Supplier;

public class RDXSCS2RestartableSimulationSession extends RDXSCS2SimulationSession
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Supplier<SimulationSession> sessionBuilder;
   private final ArrayList<String> robotsToHide = new ArrayList<>();
   private final ArrayList<String> variableWidgets = new ArrayList<>();
   private volatile boolean starting = false;
   private volatile boolean started = false;
   private final ArrayList<Runnable> destroyables = new ArrayList<>();
   private final StatelessNotification destroyedNotification = new StatelessNotification();

   public RDXSCS2RestartableSimulationSession(RDXBaseUI baseUI)
   {
      super(baseUI);
   }

   public void setSessionBuilder(Supplier<SimulationSession> sessionBuilder)
   {
      this.sessionBuilder = sessionBuilder;
   }

   @Override
   public void renderImGuiWidgets()
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
            destroySessionForRebuild();
            buildSimulation(true);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Destroy")))
         {
            destroySessionForRebuild();
         }
      }
      if (starting)
      {
         ImGui.text("Starting...");
      }

      if (started)
      {
         super.renderImGuiWidgets();
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

      startSession(sessionBuilder.get());

      for (String yoVariableName : variableWidgets)
      {
         getPlotManager().addVariableWidget(yoVariableName);
      }

      for (String robotToHide : robotsToHide)
      {
         getShowRobotMap().get(robotToHide).set(false);
      }

      starting = false;
      started = true;
   }

   public void update()
   {
      if (started)
      {
         super.update();
      }
   }

   public void destroySessionForRebuild()
   {
      if (started)
      {
         started = false;
         ThreadTools.startAsDaemon(() ->
         {
            endSession();

            for (Runnable destroyable : destroyables)
            {
               destroyable.run();
            }

            destroyedNotification.notifyOtherThread();
         }, getClass().getSimpleName() + "Destroy");
      }
   }

   public ArrayList<String> getRobotsToHide()
   {
      return robotsToHide;
   }

   public void addVariableWidget(String yoVariableName)
   {
      variableWidgets.add(yoVariableName);
   }

   public ArrayList<Runnable> getDestroyables()
   {
      return destroyables;
   }
}
