package us.ihmc.gdx.ui.footstepPlanner;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXFootstepPlannerGoalGizmo;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

import java.util.ArrayList;

public class GDXFootstepPlannerPanel extends ImGuiPanel implements RenderableProvider
{
   private final GDXFootstepPlannerGoalGizmo goalGizmo = new GDXFootstepPlannerGoalGizmo(getClass().getSimpleName());
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final GDXFootstepGraphic leftStanceFootstepGraphic;
   private final GDXFootstepGraphic rightStanceFootstepGraphic;
   private final GDXFootstepGraphic leftGoalFootstepGraphic;
   private final GDXFootstepGraphic rightGoalFootstepGraphic;
   private final Pose3D leftStanceFootPose = new Pose3D();
   private final Pose3D rightStanceFootPose = new Pose3D();
   private final FramePose3D leftGoalFootPose = new FramePose3D();
   private final FramePose3D rightGoalFootPose = new FramePose3D();
   private final ReferenceFrame goalFrame;
   private double halfIdealFootstepWidth;

   public GDXFootstepPlannerPanel(DRCRobotModel robotModel)
   {
      super("Footstep Planner");
      setRenderMethod(this::renderImGuiWidgets);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftStanceFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightStanceFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      goalFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent("goalPose", ReferenceFrame.getWorldFrame(), goalGizmo.getTransform());
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      goalGizmo.create(baseUI.get3DSceneManager().getCamera3D());
      baseUI.addImGui3DViewInputProcessor(goalGizmo::process3DViewInput);
      baseUI.get3DSceneManager().addRenderableProvider(this, GDXSceneLevel.VIRTUAL);
      leftStanceFootstepGraphic.create();
      rightStanceFootstepGraphic.create();
      leftGoalFootstepGraphic.create();
      rightGoalFootstepGraphic.create();

      halfIdealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth() / 2.0;
      leftStanceFootPose.getPosition().addY(halfIdealFootstepWidth);
      rightStanceFootPose.getPosition().subY(halfIdealFootstepWidth);
      leftStanceFootstepGraphic.setPose(leftStanceFootPose);
      rightStanceFootstepGraphic.setPose(rightStanceFootPose);
   }

   public void update()
   {
      goalFrame.update();
      leftGoalFootPose.setToZero(goalFrame);
      rightGoalFootPose.setToZero(goalFrame);
      leftGoalFootPose.getPosition().addY(halfIdealFootstepWidth);
      rightGoalFootPose.getPosition().subY(halfIdealFootstepWidth);
      leftGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      leftGoalFootstepGraphic.setPose(leftGoalFootPose);
      rightGoalFootstepGraphic.setPose(rightGoalFootPose);
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.button("Place"))
      {

      }

      goalGizmo.renderImGuiTuner();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalGizmo.getRenderables(renderables, pool);
      leftStanceFootstepGraphic.getRenderables(renderables, pool);
      rightStanceFootstepGraphic.getRenderables(renderables, pool);
      leftGoalFootstepGraphic.getRenderables(renderables, pool);
      rightGoalFootstepGraphic.getRenderables(renderables, pool);
   }
}
