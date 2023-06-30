package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class RDXManualFootstepPlacement implements RenderableProvider
{
   private static final double MAX_DISTANCE_MULTIPLIER = 3.0;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private RDXInteractableFootstep footstepBeingPlaced;
   private boolean footstepBeingPlacedIsReachable;
   private boolean modeNewlyActivated = false;
   private RDXBaseUI baseUI;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotSide currentFootStepSide;
   private RDXFootstepChecker stepChecker;
   private FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private ImGui3DViewInput latestInput;
   private RDX3DPanel primary3DPanel;
   private RDXInteractableFootstepPlan footstepPlan;
   private boolean renderTooltip = false;
   private final FramePose3D tempFramePose = new FramePose3D();
   private RDXIconTexture feetIcon;
   private RDX3DPanelTooltip tooltip;

   public void create(ROS2SyncedRobotModel syncedRobot,
                      RDXBaseUI baseUI,
                      RDXInteractableFootstepPlan footstepPlan,
                      FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      this.baseUI = baseUI;
      this.footstepPlan = footstepPlan;
      this.footstepPlannerParameters = footstepPlannerParameters;
      primary3DPanel = baseUI.getPrimary3DPanel();
      tooltip = new RDX3DPanelTooltip(primary3DPanel);
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      stepChecker = footstepPlan.getStepChecker();
      feetIcon = new RDXIconTexture("icons/feet.png");

      RDX3DPanelToolbarButton leftFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      leftFootButton.loadAndSetIcon("icons/leftFoot.png");
      leftFootButton.setTooltipText("Place left footstep");
      leftFootButton.setOnPressed(() -> createNewFootStep(RobotSide.LEFT));

      RDX3DPanelToolbarButton rightFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      rightFootButton.loadAndSetIcon("icons/rightFoot.png");
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
         tooltip.setInput(latestInput);

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
                                          footstepPlan.getNumberOfFootsteps());

         // Get the warnings and flash if the footstep's placement isn't okay
         ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
         footstepBeingPlaced.flashFootstepWhenBadPlacement(temporaryReasons.get(temporaryReasons.size() - 1));

         footstepBeingPlacedIsReachable = isFootstepBeingPlacedReachable();

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
      if (footstepBeingPlacedIsReachable)
      {
         // If safe to place footstep
         RDXInteractableFootstep addedStep = footstepPlan.getNextFootstep();
         addedStep.copyFrom(baseUI, footstepBeingPlaced);
         // Switch sides
         currentFootStepSide = currentFootStepSide.getOppositeSide();
         createNewFootStep(currentFootStepSide);
      }
      else
      {
         // If not safe print message and abort footstep placement
         LogTools.info("Footstep Rejected, too far from previous foot... not placing footstep");
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Footstep Planning:");
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
      ImGui.sameLine();
      if (imgui.ImGui.button(labels.get("Delete Last")) || (ImGui.getIO().getKeyCtrl() && ImGui.isKeyPressed('Z')))
      {
         footstepPlan.removeLastStep();
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Ctrl + Z");
   }

   private void renderTooltips()
   {
      if (renderTooltip)
      {
         if (footstepBeingPlacedIsReachable)
         {
            tooltip.render("Right click to exit.");
         }
         else
         {
            tooltip.render("Footstep out of reach.\nRight click to exit.");
         }
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
      {
         footstepBeingPlaced.update();
         footstepBeingPlaced.updateFootstepIndexText(footstepPlan.getNumberOfFootsteps());
      }
   }

   public void exitPlacement()
   {
      footstepBeingPlaced = null;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      modeNewlyActivated = true;
      RigidBodyTransformReadOnly latestFootstepTransform = footstepPlan.getLastFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();

      footstepBeingPlaced = new RDXInteractableFootstep(baseUI, footstepSide, footstepPlan.getNumberOfFootsteps(), null);
      currentFootStepSide = footstepSide;

      // Set the yaw of the new footstep to the yaw of the previous footstep
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      LibGDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().setToYawOrientation(latestFootstepYaw);
      footstepBeingPlaced.updatePose(tempFramePose);
   }

   public void squareUp()
   {
      MovingReferenceFrame midFootZUpGroundFrame = syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame();
      FramePose3D leftFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlan.getLastFootstepTransform(RobotSide.LEFT));
      leftFootPose.changeFrame(midFootZUpGroundFrame);
      FramePose3D rightFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlan.getLastFootstepTransform(RobotSide.RIGHT));
      rightFootPose.changeFrame(midFootZUpGroundFrame);
      RobotSide furthestForwardFootstep = leftFootPose.getTranslationX() < rightFootPose.getTranslationX() ? RobotSide.RIGHT : RobotSide.LEFT;
      MovingReferenceFrame soleFrame = syncedRobot.getReferenceFrames().getSoleFrame(furthestForwardFootstep);
      footstepBeingPlaced = new RDXInteractableFootstep(baseUI, furthestForwardFootstep.getOppositeSide(), footstepPlan.getNumberOfFootsteps(), null);
      tempFramePose.set(soleFrame.getTransformToRoot());
      tempFramePose.getPosition()
                   .addY(furthestForwardFootstep.negateIfLeftSide(
                         Math.cos(soleFrame.getTransformToRoot().getRotation().getYaw()) * footstepPlannerParameters.getIdealSideStepWidth()));
      tempFramePose.getPosition()
                   .addX(furthestForwardFootstep.negateIfRightSide(
                         Math.sin(soleFrame.getTransformToRoot().getRotation().getYaw()) * footstepPlannerParameters.getIdealSideStepWidth()));
      footstepBeingPlaced.updatePose(tempFramePose);
      placeFootstep();
      exitPlacement();
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

   private boolean isFootstepBeingPlacedReachable()
   {
      FramePose3D previousFootstepPose = new FramePose3D();
      // Find the previous footstep of the opposite side of the footstep being placed
      int i = footstepPlan.getNumberOfFootsteps() - 1;
      while (i >= 0 && footstepPlan.getFootsteps().get(i).getFootstepSide() == footstepBeingPlaced.getFootstepSide())
      {
         --i;
      }
      if (i >= 0)
      {
         previousFootstepPose.setIncludingFrame(footstepPlan.getFootsteps().get(i).getFootPose());
      }
      else
      {
         previousFootstepPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(currentFootStepSide.getOppositeSide()));
      }

      boolean isReachable = footstepBeingPlaced.getFootPose().getPositionDistance(previousFootstepPose) < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaximumStepReach();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() > -MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();

      return isReachable;
   }
}
