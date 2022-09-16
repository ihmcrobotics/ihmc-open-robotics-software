package us.ihmc.gdx.ui.affordances;

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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.ui.GDX3DPanelToolbarButton;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class GDXManualFootstepPlacement implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private GDXInteractableFootstep footstepBeingPlaced;
   private int footstepIndex = -1;
   private GDXImGuiBasedUI baseUI;
   private RobotSide currentFootStepSide;
   private GDXFootstepChecker stepChecker;
   private ImGui3DViewInput latestInput;
   private GDX3DPanel primary3DPanel;
   private GDXInteractableFootstepPlan footstepPlan;
   private boolean renderTooltip = false;
   private final FramePose3D tempFramePose = new FramePose3D();

   private GDXIconTexture feetIcon;

   public void create(GDXImGuiBasedUI baseUI, GDXInteractableFootstepPlan footstepPlan)
   {
      this.baseUI = baseUI;
      this.footstepPlan = footstepPlan;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      stepChecker = footstepPlan.getStepChecker();
      feetIcon = new GDXIconTexture("icons/feet.png");

      GDX3DPanelToolbarButton leftFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      leftFootButton.loadAndSetIcon("icons/leftFoot_depress.png");
      leftFootButton.setTooltipText("Place left footstep");
      leftFootButton.setOnPressed(() -> createNewFootStep(RobotSide.LEFT));

      GDX3DPanelToolbarButton rightFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      rightFootButton.loadAndSetIcon("icons/rightFoot_depress.png");
      rightFootButton.setTooltipText("Place right footstep");
      rightFootButton.setOnPressed(() -> createNewFootStep(RobotSide.RIGHT));

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);

//      baseUI.getVRManager()
   }

   public void update()
   {
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.update();
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
         GDXTools.toGDX(pickPointInWorld, footstepBeingPlaced.getFootstepModelInstance().transform);

         footstepBeingPlaced.setGizmoPose(pickPointInWorld.getX(),
                                          pickPointInWorld.getY(),
                                          pickPointInWorld.getZ(),
                                          footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());

         footstepBeingPlaced.getBoundingSphere().getPosition().set(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ());

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
               RigidBodyTransform latestFootstepTransform = footstepBeingPlaced.getFootTransformInWorld();
               double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();
               tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
               GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
               tempFramePose.set(rigidBodyTransform);
               tempFramePose.getOrientation().set(new RotationMatrix(latestFootstepYaw + deltaYaw, 0.0, 0.0));
               tempFramePose.get(footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
               footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();
            }
         }

         RigidBodyTransform candidateStepTransform = new RigidBodyTransform();
         candidateStepTransform.getTranslation().set(pickPointInWorld);
         candidateStepTransform.getRotation().setToYawOrientation(getFootstepBeingPlacedOrLastFootstepPlaced().getYaw());

         stepChecker.checkValidSingleStep(footstepPlan.getFootsteps(),
                                          candidateStepTransform,
                                          currentFootStepSide,
                                          footstepPlan.getFootsteps().size());

         // Get the warnings and flash if the footstep's placement isn't okay
         ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
         footstepBeingPlaced.flashFootstepWhenBadPlacement(temporaryReasons.get(temporaryReasons.size() - 1));

         // When left button clicked and released.
         if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
         {
            footstepIndex++;
            GDXInteractableFootstep addedStep = footstepPlan.getFootsteps().add();
            addedStep.copyFrom(baseUI, footstepBeingPlaced);
            // Switch sides
            currentFootStepSide = currentFootStepSide.getOppositeSide();
            createNewFootStep(currentFootStepSide);
         }

         if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            exitPlacement();
         }
      }
   }

   public boolean renderImGuiWidgets()
   {
      //      ImGui.text("Manual footstep placement:");
      boolean modeActivated = false;
      ImGui.image(feetIcon.getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Left")) || ImGui.isKeyPressed('R'))
      {
         modeActivated = true;
         createNewFootStep(RobotSide.LEFT);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: R");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Right")) || ImGui.isKeyPressed('T'))
      {
         modeActivated = true;
         createNewFootStep(RobotSide.RIGHT);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: T");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Cancel")) || ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         exitPlacement();
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Escape");
      return modeActivated;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.getVirtualRenderables(renderables, pool);
      }
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

   public void exitPlacement()
   {
      footstepBeingPlaced = null;
      footstepIndex = -1;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      RigidBodyTransform latestFootstepTransform = footstepPlan.getLastFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();

      footstepBeingPlaced = new GDXInteractableFootstep(baseUI, footstepSide, footstepIndex);
      currentFootStepSide = footstepSide;

      // Set the yaw of the new footstep to the yaw of the previous footstep
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().set(new RotationMatrix(latestFootstepYaw, 0.0, 0.0));
      tempFramePose.get(footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
      footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();
   }

   /**
    * Returns future footstep currently being placed. If you are not placing a footstep currently, it will return last footstep from list.
    * Does NOT return footsteps that you already walked on.
    */
   public GDXInteractableFootstep getFootstepBeingPlacedOrLastFootstepPlaced()
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
}
