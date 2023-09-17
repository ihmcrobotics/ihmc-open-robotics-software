package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFootstep;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.function.BooleanSupplier;

/**
 * This class is a fully mutable and transitive interactable representation of FootstepActionData.
 */
public class RDXFootstepAction
{
   private final ReferenceFrameSupplier planFrameSupplier;
   private final RDXBaseUI baseUI;
   private final DRCRobotModel robotModel;
   private final BooleanSupplier planSelected;
   private FootstepActionData actionData;
   private RDXInteractableFootstep interactableFootstep;
   private RDXFootstepGraphic flatFootstepGraphic;

   public RDXFootstepAction(ReferenceFrameSupplier planFrameSupplier,
                            RDXBaseUI baseUI,
                            DRCRobotModel robotModel,
                            BooleanSupplier planSelected)
   {
      this.planFrameSupplier = planFrameSupplier;
      this.baseUI = baseUI;
      this.robotModel = robotModel;
      this.planSelected = planSelected;
   }

   public void update(FootstepActionData actionData, int stepIndex)
   {
      this.actionData = actionData;

      if (interactableFootstep == null
       || stepIndex != interactableFootstep.getIndex()
       || actionData.getSide() != interactableFootstep.getFootstepSide())
      {
         recreateGraphics(actionData.getSide(), stepIndex);
      }

      ReferenceFrame updatedPlanFrame = planFrameSupplier.get();
      if (updatedPlanFrame != getFootstepFrame().getParent())
      {
         getGizmo().changeParentFrameWithoutMoving(updatedPlanFrame);
      }

      getGizmo().update();

      if (getGizmo().getGizmoModifiedByUser().poll())
         actionData.getSolePose().set(getGizmo().getTransformToParent()); // Update action data based on user input
      else
      {
         getGizmo().getTransformToParent().set(actionData.getSolePose()); // Update gizmo in case action data changes
         getGizmo().update();
      }

      interactableFootstep.update();

      flatFootstepGraphic.setPose(getGizmo().getPose());
   }

   /** The only reason we have to recreate these is because the side isn't mutable. */
   private void recreateGraphics(RobotSide side, int stepIndex)
   {
      interactableFootstep = new RDXInteractableFootstep(baseUI, side, stepIndex, null);
      flatFootstepGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
      flatFootstepGraphic.create();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (planSelected.getAsBoolean())
      {
         interactableFootstep.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (planSelected.getAsBoolean())
      {
         interactableFootstep.process3DViewInput(input, false);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (planSelected.getAsBoolean())
         interactableFootstep.getVirtualRenderables(renderables, pool);
      else
         flatFootstepGraphic.getRenderables(renderables, pool);
   }

   public FootstepActionData getActionData()
   {
      return actionData;
   }

   public ReferenceFrame getFootstepFrame()
   {
      return getGizmo().getGizmoFrame();
   }

   public RDXPose3DGizmo getGizmo()
   {
      return interactableFootstep.getSelectablePose3DGizmo().getPoseGizmo();
   }
}
