package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayDeque;
import java.util.ArrayList;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class RDXManualFootstepPlacement implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private RDXInteractableFootstep footstepBeingPlaced;
   private int footstepIndex = -1;
   private boolean modeNewlyActivated = false;
   private RDXBaseUI baseUI;
   private RobotSide currentFootStepSide;
   private RDXFootstepChecker stepChecker;
   private ImGui3DViewInput latestInput;
   private RDX3DPanel primary3DPanel;
   private RDXInteractableFootstepPlan footstepPlan;
   private boolean renderTooltip = false;
   private final ArrayDeque<RDXInteractableFootstep> newlyPlacedFootsteps = new ArrayDeque<>();
   private final FramePose3D tempFramePose = new FramePose3D();
   private RDXIconTexture feetIcon;

   public void create(RDXBaseUI baseUI, RDXInteractableFootstepPlan footstepPlan)
   {
      this.baseUI = baseUI;
      this.footstepPlan = footstepPlan;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      stepChecker = footstepPlan.getStepChecker();
      feetIcon = new RDXIconTexture("icons/feet.png");

      RDX3DPanelToolbarButton leftFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      leftFootButton.loadAndSetIcon("icons/leftFoot_depress.png");
      leftFootButton.setTooltipText("Place left footstep");
      leftFootButton.setOnPressed(() -> createNewFootStep(RobotSide.LEFT));

      RDX3DPanelToolbarButton rightFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      rightFootButton.loadAndSetIcon("icons/rightFoot_depress.png");
      rightFootButton.setTooltipText("Place right footstep");
      rightFootButton.setOnPressed(() -> createNewFootStep(RobotSide.RIGHT));
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      renderTooltip = false;

      if (footstepBeingPlaced != null)
         footstepBeingPlaced.calculate3DViewPick(input);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;

      renderTooltip = footstepBeingPlaced != null;

      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.process3DViewInput(input, true);

         Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
         renderTooltip = true;

         // Set position of modelInstance, selectablePose3DGizmo,
         // and the sphere used in stepCheckIsPointInsideAlgorithm all to the pointInWorld that the cursor is at
         LibGDXTools.toLibGDX(pickPointInWorld, footstepBeingPlaced.getFootstepModelInstance().transform);

         footstepBeingPlaced.setGizmoPose(pickPointInWorld.getX(),
                                          pickPointInWorld.getY(),
                                          pickPointInWorld.getZ(),
                                          footstepBeingPlaced.getFootPose());

         // Adjust footstep yaw while placing with Ctrl + Mouse Scroll Up/Down
         double deltaYaw = 0.0;
         boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
         if (ctrlHeld)
         {
            float dScroll = input.getMouseWheelDelta();
            if (dScroll > 0.0)
            {
               deltaYaw = 0.03 * Math.PI;
            }
            else if (dScroll < 0.0)
            {
               deltaYaw = -0.03 * Math.PI;
            }
            if (deltaYaw != 0.0)
            {
               FramePose3DReadOnly latestFootstepPose = footstepBeingPlaced.getFootPose();
               double latestFootstepYaw = latestFootstepPose.getRotation().getYaw();
               tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
               LibGDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
               tempFramePose.set(rigidBodyTransform);
               tempFramePose.getOrientation().setToYawOrientation(latestFootstepYaw + deltaYaw);
               footstepBeingPlaced.updatePose(tempFramePose);
            }
         }

         FramePose3D candidateStepPose = new FramePose3D();
         candidateStepPose.getPosition().set(pickPointInWorld);
         candidateStepPose.getRotation().setToYawOrientation(getFootstepBeingPlacedOrLastFootstepPlaced().getYaw());

         stepChecker.checkValidSingleStep(footstepPlan.getFootsteps(),
                                          candidateStepPose,
                                          currentFootStepSide,
                                          footstepPlan.getFootsteps().size());

         // Get the warnings and flash if the footstep's placement isn't okay
         ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
         footstepBeingPlaced.flashFootstepWhenBadPlacement(temporaryReasons.get(temporaryReasons.size() - 1));

         // When left button clicked and released.
         if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
         {
            placeFootstep();
         }

         if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            exitPlacement();
         }
      }
   }

   private void placeFootstep()
   {
      footstepIndex++;
      RDXInteractableFootstep addedStep = footstepPlan.getFootsteps().add();
      addedStep.copyFrom(baseUI, footstepBeingPlaced);
      // Switch sides
      currentFootStepSide = currentFootStepSide.getOppositeSide();
      createNewFootStep(currentFootStepSide);
   }

   public void renderImGuiWidgets()
   {
//      ImGui.text("Manual footstep placement:");
      ImGui.image(feetIcon.getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Left")) || ImGui.isKeyPressed('R'))
      {
         createNewFootStep(RobotSide.LEFT);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: R");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Right")) || ImGui.isKeyPressed('T'))
      {
         createNewFootStep(RobotSide.RIGHT);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: T");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Cancel")) || ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         exitPlacement();
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Escape");
   }

   private void renderTooltips()
   {
      if (renderTooltip)
      {
         float offsetX = 10.0f;
         float offsetY = 10.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 150.0f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(),
                       ImGuiTools.getSmallFont().getFontSize(),
                       drawStartX + 5.0f,
                       drawStartY + 2.0f,
                       Color.WHITE.toIntBits(),
                       "Right click to exit");
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.getVirtualRenderables(renderables, pool);
      }
   }

   public void update()
   {
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.update();
   }

   public void exitPlacement()
   {
      footstepBeingPlaced = null;
      footstepIndex = -1;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      modeNewlyActivated = true;
      RigidBodyTransformReadOnly latestFootstepTransform = footstepPlan.getLastFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();

      footstepBeingPlaced = new RDXInteractableFootstep(baseUI, footstepSide, footstepIndex, null);
      currentFootStepSide = footstepSide;

      // Set the yaw of the new footstep to the yaw of the previous footstep
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      LibGDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().setToYawOrientation(latestFootstepYaw);
      footstepBeingPlaced.updatePose(tempFramePose);
   }

   public boolean pollIsModeNewlyActivated()
   {
      boolean modeNewlyActivatedReturn = modeNewlyActivated;
      modeNewlyActivated = false;
      return modeNewlyActivatedReturn;
   }

   /**
    * Returns future footstep currently being placed. If you are not placing a footstep currently, it will return last footstep from list.
    * Does NOT return footsteps that you already walked on.
    */
   public RDXInteractableFootstep getFootstepBeingPlacedOrLastFootstepPlaced()
   {
      if (footstepBeingPlaced != null)
      {
         return footstepBeingPlaced;
      }
      else
      {
         return footstepPlan.getLastFootstep();
      }
   }

   public boolean isPlacingFootstep()
   {
      return footstepBeingPlaced != null;
   }
}
