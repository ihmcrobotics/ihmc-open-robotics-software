package us.ihmc.rdx.ui;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.behaviors.simulation.EnvironmentInitialSetup;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.processes.RestartableProcess;

import java.util.ArrayList;

public abstract class RDXProcessManagerPanel extends RDXPanel
{
   private static final String PANEL_NAME = "Process Manager";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   protected final ImInt robotTarget = new ImInt(1);
   protected final String[] robotTargets = new String[] {"Real robot", "Simulation"};
   protected final ImBoolean logToFile = new ImBoolean(false);

   protected final ArrayList<RestartableProcess> processes = new ArrayList<>();

   protected final EnvironmentInitialSetup environmentInitialSetup;

   public RDXProcessManagerPanel(Pose3DReadOnly startingPose)
   {
      this(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::flatGround,
                                       startingPose.getZ(),
                                       Math.toRadians(0.0),
                                       startingPose.getX(),
                                       startingPose.getY()));
   }

   public RDXProcessManagerPanel(EnvironmentInitialSetup environmentInitialSetup)
   {
      super(PANEL_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      this.environmentInitialSetup = environmentInitialSetup;
   }

   private RobotTarget getRobotTarget()
   {
      return RobotTarget.values()[robotTarget.get()];
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushItemWidth(150.0f);
      ImGui.text("Robot target:");
      ImGui.sameLine();
      ImGui.combo(labels.get("RobotTarget"), robotTarget, robotTargets, robotTargets.length);
      ImGui.text("Robot version:");
      ImGui.sameLine();
      ImGui.combo(labels.get("RobotVersion"), getRobotVersion(), getRobotVersions(), getRobotVersions().length);
      ImGui.popItemWidth();

      ImGui.text("Simulation:");
      ImGui.sameLine();

      ImGui.checkbox(labels.get("Log to file"), logToFile);

      for (RestartableProcess process : processes)
      {
         process.renderImGuiWidgets();
      }
   }

   protected abstract ImInt getRobotVersion();

   protected abstract String[] getRobotVersions();

   protected abstract DRCRobotModel getRobotModel();

   public void dispose()
   {
      // destroy em all just in case
      for (RestartableProcess process : processes)
      {
         process.destroy();
      }
   }
}
