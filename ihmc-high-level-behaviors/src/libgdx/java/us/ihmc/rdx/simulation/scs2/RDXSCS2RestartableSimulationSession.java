package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;

import java.util.ArrayList;
import java.util.function.Supplier;

public class RDXSCS2RestartableSimulationSession extends RDXSCS2SimulationSession
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Supplier<SimulationSession> sessionBuilder;
   private final ArrayList<String> robotsToHide = new ArrayList<>();
   private final ArrayList<String> variableWidgets = new ArrayList<>();
   private volatile boolean starting = false;
   private final ArrayList<Runnable> destroyables = new ArrayList<>();

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
      if (!starting && !isSessionThreadRunning())
      {
         if (ImGui.button(labels.get("Build simulation")))
         {
            buildSimulation();
         }
      }
      if (isSessionThreadRunning())
      {
         if (ImGui.button(labels.get("Rebuild simulation")))
         {
            destroySessionForRebuild();
            buildSimulation();
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

      if (isSessionThreadRunning())
      {
         super.renderImGuiWidgets();
      }
   }

   public void buildSimulation()
   {
      starting = true;
      waitForSessionToBeStopped();

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
   }

   public void update()
   {
      if (isSessionThreadRunning())
      {
         super.update();
      }
   }

   public void destroySessionForRebuild()
   {
      if (isSessionThreadRunning())
      {
         stopSession();

         for (Runnable destroyable : destroyables)
         {
            destroyable.run();
         }
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
