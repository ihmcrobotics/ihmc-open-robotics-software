package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningRemoteTask;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class RDXContinuousPlanningUI implements RenderableProvider
{
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final ImBoolean enableContinuousPlanner = new ImBoolean(false);
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ContinuousPlanningRemoteTask continuousPlanningRemoteTask;
   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private RDXPanel panel;
   private ROS2SyncedRobotModel syncedRobot;
   private SideDependentList<RDXFootstepGraphic> goalFootstepGraphics;
   private SideDependentList<RDXFootstepGraphic> startFootstepGraphics;

   public RDXContinuousPlanningUI(String name,
                                  ContinuousPlanningRemoteTask continuousPlanningRemoteTask,
                                  PerceptionConfigurationParameters perceptionConfigurationParameters,
                                  ROS2SyncedRobotModel syncedRobot)
   {
      panel = new RDXPanel(name, this::renderImGuiWidgets);
      this.continuousPlanningRemoteTask = continuousPlanningRemoteTask;
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
      this.syncedRobot = syncedRobot;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = syncedRobot.getRobotModel()
                                                                                     .getContactPointParameters()
                                                                                     .getControllerFootGroundContactPoints();
      goalFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));
      startFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(contactPoints, RobotSide.LEFT), new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT));

      goalFootstepGraphics.get(RobotSide.RIGHT).setColor(new Color(1.0f, 1.0f, 1.0f, 0.5f));
      goalFootstepGraphics.get(RobotSide.RIGHT).create();

      goalFootstepGraphics.get(RobotSide.LEFT).setColor(new Color(0.8f, 0.8f, 0.8f, 0.5f));
      goalFootstepGraphics.get(RobotSide.LEFT).create();

      startFootstepGraphics.get(RobotSide.RIGHT).setColor(new Color(0.6f, 0.6f, 0.6f, 0.5f));
      startFootstepGraphics.get(RobotSide.RIGHT).create();

      startFootstepGraphics.get(RobotSide.LEFT).setColor(new Color(0.2f, 0.2f, 0.2f, 0.5f));
      startFootstepGraphics.get(RobotSide.LEFT).create();
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox("Enable Continuous Planner", enableContinuousPlanner);
      if (perceptionConfigurationParameters != null)
      {
         perceptionConfigurationParameters.setActiveMapping(enableContinuousPlanner.get());
      }
   }

   public void render()
   {
      for (RobotSide side : RobotSide.values)
      {
         startFootstepGraphics.get(side).setPose(continuousPlanningRemoteTask.getStartPose().get(side));
         goalFootstepGraphics.get(side).setPose(continuousPlanningRemoteTask.getGoalPose().get(side));
      }
   }

   public RDXPanel getImGuiPanel()
   {
      return panel;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         goalFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         goalFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         startFootstepGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }

   public void destroy()
   {
      footstepPlanGraphic.destroy();
      goalFootstepGraphics.get(RobotSide.LEFT).destroy();
      goalFootstepGraphics.get(RobotSide.RIGHT).destroy();
      startFootstepGraphics.get(RobotSide.LEFT).destroy();
      startFootstepGraphics.get(RobotSide.RIGHT).destroy();
   }
}
