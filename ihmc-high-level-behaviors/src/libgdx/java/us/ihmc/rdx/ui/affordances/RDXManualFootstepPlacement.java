package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
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
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
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
   private RDXInteractableFootstepPlan footstepPlan;
   private boolean renderTooltip = false;
   private final FramePose3D tempFramePose = new FramePose3D();
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
      tooltip = new RDX3DPanelTooltip(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::renderTooltips);
      stepChecker = footstepPlan.getStepChecker();

      RDX3DPanelToolbarButton leftFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      leftFootButton.loadAndSetIcon("icons/leftFoot.png");
      leftFootButton.setTooltipText("Place left footstep (R)");
      leftFootButton.setOnPressed(() -> createNewFootstep(RobotSide.LEFT));

      RDX3DPanelToolbarButton rightFootButton = baseUI.getPrimary3DPanel().addToolbarButton();
      rightFootButton.loadAndSetIcon("icons/rightFoot.png");
      rightFootButton.setTooltipText("Place right footstep (T)");
      rightFootButton.setOnPressed(() -> createNewFootstep(RobotSide.RIGHT));

      RDXBaseUI.getInstance().getKeyBindings().register("Place left footstep", "R");
      RDXBaseUI.getInstance().getKeyBindings().register("Place right footstep", "T");
      RDXBaseUI.getInstance().getKeyBindings().register("Delete last interactable footstep", "Delete");
      RDXBaseUI.getInstance().getKeyBindings().register("Cancel footstep placement", "Escape");
   }

   public void update()
   {
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.update();
         footstepBeingPlaced.updateFootstepIndexText(footstepPlan.getNumberOfFootsteps());

         // Deselect all footsteps since we are placing a new one
         for (int i = 0; i < footstepPlan.getNumberOfFootsteps(); i++)
            footstepPlan.getFootsteps().get(i).getSelectablePose3DGizmo().setSelected(false);
      }
   }

   public void renderImGuiWidgets()
   {
      boolean panel3DIsHovered = latestInput != null && latestInput.isWindowHovered();

      ImGui.text("Footstep Planning:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Left")) || (panel3DIsHovered && ImGui.isKeyPressed('R')))
      {
         createNewFootstep(RobotSide.LEFT);
      }
      ImGuiTools.previousWidgetTooltip("R");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Right")) || (panel3DIsHovered && ImGui.isKeyPressed('T')))
      {
         createNewFootstep(RobotSide.RIGHT);
      }
      ImGuiTools.previousWidgetTooltip("T");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Square Up")))
      {
         squareUpFootstep();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Cancel")) || ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         exitPlacement();
      }
      ImGuiTools.previousWidgetTooltip("Escape");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Delete Last")) || ImGui.isKeyPressed(ImGuiTools.getDeleteKey()))
      {
         footstepPlan.removeLastStep();
      }
      ImGuiTools.previousWidgetTooltip("Delete");
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.calculateVRPick(vrContext);
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.processVRInput(vrContext);
      }
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

         // This allows us to check the current footstep being placed and flash that footstep if its unreasonable
         FramePose3DReadOnly candidateStepPose = getFootstepBeingPlacedPoseORLastFootstepPose();
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

   public void placeFootstep()
   {
      if (footstepBeingPlacedIsReachable)
      {
         // If safe to place footstep
         RDXInteractableFootstep addedStep = footstepPlan.getNextFootstep();
         addedStep.copyFrom(baseUI, footstepBeingPlaced);
         // Switch sides
         currentFootStepSide = currentFootStepSide.getOppositeSide();
         createNewFootstep(currentFootStepSide);
      }
      else
      {
         // If not safe print message and abort footstep placement
         LogTools.info("Footstep Rejected, too far from previous foot... not placing footstep");
      }
   }

   public void forcePlaceFootstep()
   {
      footstepBeingPlacedIsReachable = true;
      placeFootstep();
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

   public void exitPlacement()
   {
      footstepBeingPlaced = null;
   }

   public void createNewFootstep(RobotSide footstepSide)
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

   public void setFootstepPose(FramePose3DReadOnly poseToSet)
   {
      poseToSet.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      footstepBeingPlaced.updatePose(poseToSet);
   }

   public void squareUpFootstep()
   {
      footstepPlan.clear();
      ReferenceFrame leftFootFrame = syncedRobot.getReferenceFrames().getFootFrame(RobotSide.LEFT);
      FramePose3D rightFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                  syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
      rightFootPose.changeFrame(leftFootFrame);
      RobotSide furthestForwardFootstep = rightFootPose.getTranslationX() > 0 ? RobotSide.RIGHT : RobotSide.LEFT;
      MovingReferenceFrame furthestForwardSoleFrame = syncedRobot.getReferenceFrames().getSoleFrame(furthestForwardFootstep);
      footstepBeingPlaced = new RDXInteractableFootstep(baseUI, furthestForwardFootstep.getOppositeSide(), footstepPlan.getNumberOfFootsteps(), null);
      tempFramePose.setToZero(furthestForwardSoleFrame);
      tempFramePose.getTranslation().addY(furthestForwardFootstep.negateIfLeftSide(footstepPlannerParameters.getIdealFootstepWidth()));
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
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
   public FramePose3DReadOnly getFootstepBeingPlacedPoseORLastFootstepPose()
   {
      if (footstepBeingPlaced != null)
      {
         return footstepBeingPlaced.getFootPose();
      }
      else
      {
         return footstepPlan.getLastFootstep().getFootPose();
      }
   }

   public boolean isPlacingFootstep()
   {
      return footstepBeingPlaced != null;
   }

   private boolean isFootstepBeingPlacedReachable()
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      previousFootstepPose.set(stepChecker.getPreviousFootstepOnOppositeSide(footstepPlan.getFootsteps(),
                                                                             footstepPlan.getNumberOfFootsteps(),
                                                                             footstepBeingPlaced.getFootstepSide()));

      boolean isReachable = footstepBeingPlaced.getFootPose().getPositionDistance(previousFootstepPose) < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaximumStepReach();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() > -MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();

      return isReachable;
   }

   public void walkFromSteps()
   {
      footstepPlan.walkFromSteps();
   }
}