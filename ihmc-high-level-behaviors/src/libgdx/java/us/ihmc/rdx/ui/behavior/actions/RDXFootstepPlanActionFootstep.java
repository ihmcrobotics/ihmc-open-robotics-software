package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionFootstepDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionFootstepState;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFootstep;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;

/**
 * This class is a fully mutable and transitive interactable representation of FootstepActionDefinition.
 */
public class RDXFootstepPlanActionFootstep
{
   private final RDXFootstepPlanAction footstepPlan;
   private final RDXBaseUI baseUI;
   private final DRCRobotModel robotModel;
   private final FootstepPlanActionFootstepState state;
   private final FootstepPlanActionFootstepDefinition definition;
   private RDXInteractableFootstep interactableFootstep;
   private RDXFootstepGraphic flatFootstepGraphic;

   public RDXFootstepPlanActionFootstep(RDXBaseUI baseUI,
                                        DRCRobotModel robotModel,
                                        RDXFootstepPlanAction footstepPlan,
                                        FootstepPlanActionFootstepState state)
   {
      this.footstepPlan = footstepPlan;
      this.baseUI = baseUI;
      this.robotModel = robotModel;

      this.state = state;
      definition = state.getDefinition();
   }

   public void update()
   {
      state.update();

      if (interactableFootstep == null
          || state.getIndex() != interactableFootstep.getIndex()
          || definition.getSide() != interactableFootstep.getFootstepSide())
      {
         recreateGraphics();
      }

      if (state.getSoleFrame().isChildOfWorld())
      {
         if (getGizmo().getGizmoFrame().getParent() != state.getSoleFrame().getReferenceFrame().getParent())
         {
            getGizmo().changeParentFrameWithoutMoving(state.getSoleFrame().getReferenceFrame().getParent());
         }

         getGizmo().update();

         if (getGizmo().getGizmoModifiedByUser().poll())
         {
            definition.getSoleToPlanFrameTransform().getValue().set(getGizmo().getTransformToParent()); // Update action data based on user input
         }
         else
         {
            getGizmo().getTransformToParent().set(definition.getSoleToPlanFrameTransform().getValueReadOnly()); // Update gizmo in case action data changes
            getGizmo().update();
         }

         interactableFootstep.update();

         flatFootstepGraphic.setPoseFromReferenceFrame(getState().getSoleFrame().getReferenceFrame());
      }
   }

   /** The only reason we have to recreate these is because the side isn't mutable. */
   private void recreateGraphics()
   {
      interactableFootstep = new RDXInteractableFootstep(baseUI, definition.getSide(), state.getIndex(), null);
      flatFootstepGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), definition.getSide());
      flatFootstepGraphic.create();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getSoleFrame().isChildOfWorld() && footstepPlan.getShowAdjustmentInteractables().get())
      {
         interactableFootstep.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getSoleFrame().isChildOfWorld() && footstepPlan.getShowAdjustmentInteractables().get())
      {
         interactableFootstep.process3DViewInput(input, false);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getSoleFrame().isChildOfWorld())
      {
         if (footstepPlan.getShowAdjustmentInteractables().get())
            interactableFootstep.getVirtualRenderables(renderables, pool);
         else
            flatFootstepGraphic.getRenderables(renderables, pool);
      }
   }

   private RDXPose3DGizmo getGizmo()
   {
      return interactableFootstep.getSelectablePose3DGizmo().getPoseGizmo();
   }

   public FootstepPlanActionFootstepDefinition getDefinition()
   {
      return definition;
   }

   public FootstepPlanActionFootstepState getState()
   {
      return state;
   }
}
