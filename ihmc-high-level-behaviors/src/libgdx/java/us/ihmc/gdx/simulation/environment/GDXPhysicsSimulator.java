package us.ihmc.gdx.simulation.environment;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;

public class GDXPhysicsSimulator
{
   private final SimulationSession simulationSession;
   private final ImBoolean runAtRealtimeRate;
   private final ImDouble playbackRealtimeRate;
   private final ImGuiPanel controlPanel = new ImGuiPanel("Physics Simulator", this::renderImGuiWidgets);

   public GDXPhysicsSimulator()
   {
      simulationSession = new SimulationSession();
      runAtRealtimeRate = new ImBoolean(simulationSession.getRunAtRealTimeRate());
      playbackRealtimeRate = new ImDouble(simulationSession.getPlaybackRealTimeRate());
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.radioButton("Simulate", simulationSession.getActiveMode() == SessionMode.RUNNING))
      {
         simulationSession.setSessionMode(SessionMode.RUNNING);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Pause", simulationSession.getActiveMode() == SessionMode.PAUSE))
      {
         simulationSession.setSessionMode(SessionMode.PAUSE);
      }
      ImGui.sameLine();
      if (ImGui.radioButton("Playback", simulationSession.getActiveMode() == SessionMode.PLAYBACK))
      {
         simulationSession.setSessionMode(SessionMode.PLAYBACK);
      }
      if (ImGui.checkbox("Run at real-time rate", runAtRealtimeRate))
      {
         simulationSession.submitRunAtRealTimeRate(runAtRealtimeRate.get());
      }
      ImGui.pushItemWidth(100.0f);
      if (ImGui.inputDouble("Playback real-time rate", playbackRealtimeRate))
      {
         simulationSession.submitPlaybackRealTimeRate(playbackRealtimeRate.get());
      }
      ImGui.popItemWidth();
   }

   public SimulationSession getSession()
   {
      return simulationSession;
   }

   public ImGuiPanel getControlPanel()
   {
      return controlPanel;
   }
}
