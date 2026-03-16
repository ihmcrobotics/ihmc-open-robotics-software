package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;

/** UI for the behavior footstep plan action's modifiable goal footstep, used by the footstep planner. */
public class RDXWalkActionGoalFootstep
{
   private final RobotSide side;
   private final WalkActionDefinition definition;
   private final WalkActionState state;

   private final RDXSelectablePose3DGizmo gizmo;
   private final RDXFootstepGraphic graphic;

   public RDXWalkActionGoalFootstep(RDXBaseUI baseUI,
                                            RobotSide side,
                                            WalkActionDefinition definition,
                                            WalkActionState state,
                                            DRCRobotModel robotModel)
   {
      this.side = side;
      this.definition = definition;
      this.state = state;

      gizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), state.getGoalFootstepToGoalTransform(side));
      gizmo.create(baseUI.getPrimary3DPanel());
      gizmo.getPoseGizmo().getCenterSphereToTorusRatio().set(0.3f);

      graphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
      graphic.create();
   }

   public void setParentFrame()
   {
      gizmo.getPoseGizmo().setParentFrame(state.getGoalFrame().getReferenceFrame());
   }

   public void updatePoses()
   {
      gizmo.getPoseGizmo().update();

      graphic.setPose(transform ->
      {
         transform.setToZero();
         transform.getTranslation().setX(definition.getGoalFootstepToGoalX(side).getValue());
         transform.getTranslation().setY(definition.getGoalFootstepToGoalY(side).getValue());
         transform.getRotation().setToYawOrientation(definition.getGoalFootstepToGoalYaw(side).getValue());
         state.getGoalFrame().getReferenceFrame().getTransformToRoot().transform(transform);
      });
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      gizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      gizmo.process3DViewInput(input, false);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, boolean isHighlighted)
   {
      gizmo.getVirtualRenderables(renderables, pool);
      graphic.setHighlighted(isHighlighted);
      graphic.getRenderables(renderables, pool);
   }

   public RDXSelectablePose3DGizmo getGizmo()
   {
      return gizmo;
   }
}
