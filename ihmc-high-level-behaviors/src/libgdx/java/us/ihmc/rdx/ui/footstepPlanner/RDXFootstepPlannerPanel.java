package us.ihmc.rdx.ui.footstepPlanner;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

import java.util.ArrayList;

public class RDXFootstepPlannerPanel extends RDXPanel implements RenderableProvider
{
   private final RDXPathControlRingGizmo goalGizmo = new RDXPathControlRingGizmo();
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RDXFootstepGraphic leftStanceFootstepGraphic;
   private final RDXFootstepGraphic rightStanceFootstepGraphic;
   private final RDXFootstepGraphic leftGoalFootstepGraphic;
   private final RDXFootstepGraphic rightGoalFootstepGraphic;
   private final Pose3D leftStanceFootPose = new Pose3D();
   private final Pose3D rightStanceFootPose = new Pose3D();
   private final FramePose3D leftGoalFootPose = new FramePose3D();
   private final FramePose3D rightGoalFootPose = new FramePose3D();
   private final ReferenceFrame goalFrame;
   private final FootstepPlanningModule footstepPlanner;
   private final RDXFootstepPlanGraphic foostepPlanGraphic;
   private double halfIdealFootstepWidth;

   public RDXFootstepPlannerPanel(DRCRobotModel robotModel)
   {
      super("Footstep Planner");
      setRenderMethod(this::renderImGuiWidgets);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      goalFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("goalPose", ReferenceFrame.getWorldFrame(), goalGizmo.getTransformToParent());

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      foostepPlanGraphic = new RDXFootstepPlanGraphic(contactPoints);
      addChild(goalGizmo.createTunerPanel(getClass().getSimpleName()));
   }

   public void create(RDXBaseUI baseUI)
   {
      goalGizmo.createAndSetupDefault(baseUI);
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

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setStartFootPoses(leftStanceFootPose, rightStanceFootPose);
      footstepPlannerRequest.setGoalFootPoses(leftGoalFootPose, rightGoalFootPose);
      footstepPlannerRequest.setAssumeFlatGround(true);
      try
      {
         FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
         if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() > 0)
         {
            foostepPlanGraphic.generateMeshes(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(),
                                                                                              "Footstep Planner Panel Plan"));
         }
      }
      catch (RuntimeException e)
      {

      }
      foostepPlanGraphic.update();
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.button("Place"))
      {

      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalGizmo.getRenderables(renderables, pool);
      leftStanceFootstepGraphic.getRenderables(renderables, pool);
      rightStanceFootstepGraphic.getRenderables(renderables, pool);
      leftGoalFootstepGraphic.getRenderables(renderables, pool);
      rightGoalFootstepGraphic.getRenderables(renderables, pool);
      foostepPlanGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      foostepPlanGraphic.destroy();
   }
}
