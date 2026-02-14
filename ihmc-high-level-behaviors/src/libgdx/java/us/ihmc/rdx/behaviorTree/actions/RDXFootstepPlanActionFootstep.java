package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionFootstepDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionFootstepState;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFootstep;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
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
   private RDXPose3DGizmo gizmo;
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
         recreateGraphics();

      if (state.getSoleFrame().isChildOfWorld())
      {
         if (gizmo.getGizmoFrame().getParent() != state.getSoleFrame().getReferenceFrame().getParent())
            gizmo.changeParentFrameWithoutMoving(state.getSoleFrame().getReferenceFrame().getParent());

         RDXCRDTTools.syncGizmoWithBidirectionalField(gizmo, definition.getSoleToPlanFrameTransform(), footstepPlan.getDefinition());

         interactableFootstep.update();

         flatFootstepGraphic.setPoseFromReferenceFrame(state.getSoleFrame().getReferenceFrame());
      }
   }

   /** The only reason we have to recreate these is because the side isn't mutable. */
   private void recreateGraphics()
   {
      interactableFootstep = new RDXInteractableFootstep(baseUI, definition.getSide(), state.getIndex(), null);
      gizmo = interactableFootstep.getSelectablePose3DGizmo().getPoseGizmo();
      flatFootstepGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), definition.getSide());
      flatFootstepGraphic.create();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getSoleFrame().isChildOfWorld() && footstepPlan.getEditManuallyPlacedSteps().get())
      {
         interactableFootstep.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getSoleFrame().isChildOfWorld() && footstepPlan.getEditManuallyPlacedSteps().get())
      {
         interactableFootstep.process3DViewInput(input, false);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getSoleFrame().isChildOfWorld())
      {
         if (footstepPlan.getEditManuallyPlacedSteps().get())
            interactableFootstep.getVirtualRenderables(renderables, pool);
         else
            flatFootstepGraphic.getRenderables(renderables, pool);
      }
   }

   public void updateGizmo()
   {
      gizmo.getTransformToParent().set(definition.getSoleToPlanFrameTransform().getValueReadOnly());
      interactableFootstep.update();
   }

   public RDXInteractableFootstep getInteractableFootstep()
   {
      return interactableFootstep;
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
