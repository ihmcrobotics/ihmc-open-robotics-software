package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commonWalkingControlModules.desiredFootStep.EllipticalStepPositionLimiter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.factories.OptionalFactoryField;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class RDXManualFootstepPlacement implements RenderableProvider
{
   private final static boolean USE_HEIGHTMAP = true;
   private final static boolean APPLY_REACHABLE_REGION_ELLIPTICAL_CONSTRAINT = false;
   private static final double MAX_DISTANCE_MULTIPLIER = 3.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXInteractableFootstep footstepBeingPlaced;
   private boolean footstepBeingPlacedIsReachable;
   private boolean modeNewlyActivated = false;
   private boolean needToInitializePlacementHeight = false;
   private RDXBaseUI baseUI;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotSide currentFootStepSide;
   private RDXFootstepChecker stepChecker;
   private DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters;
   private ImGui3DViewInput latestInput;
   private RDXInteractableFootstepPlan footstepPlan;
   private boolean renderTooltip = false;
   private final FramePose3D tempFramePose = new FramePose3D();
   private RDX3DPanelTooltip tooltip;
   private final ImBoolean activeAdjustmentEnabled = new ImBoolean(false);
   private TerrainMapData latestHeightMapData;
   private final OptionalFactoryField<EllipticalStepPositionLimiter> stepPositionLimiter = new OptionalFactoryField<>("ManualFootstepPlacementLimiter");

   public void create(ROS2SyncedRobotModel syncedRobot,
                      RDXBaseUI baseUI,
                      RDXInteractableFootstepPlan footstepPlan,
                      DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters)
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

         if (needToInitializePlacementHeight) // Initialize placed footstep height until scene collisions take back over
         {
            needToInitializePlacementHeight = false;
            input.setLastZCollision(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());
         }

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

         // Constrain footstep to reachable region
         if (APPLY_REACHABLE_REGION_ELLIPTICAL_CONSTRAINT)
            applyReachabilityConstraintToStep(footstepBeingPlaced.getFootPose());

         // Snap footstep to height map
         if (USE_HEIGHTMAP && latestHeightMapData != null)
         {
            double height = latestHeightMapData.getHeight(footstepBeingPlaced.getFootPose().getX(), footstepBeingPlaced.getFootPose().getY());

            if (!Double.isNaN(height))
               footstepBeingPlaced.getFootPose().setZ(height);
            else
               LogTools.warn("Could not use heightMap for footstep adjustment, since height is NaN");
         }

         // Update gizmo if we used height map or constrained footstep position
         if (USE_HEIGHTMAP || APPLY_REACHABLE_REGION_ELLIPTICAL_CONSTRAINT)
            footstepBeingPlaced.setGizmoPose(footstepBeingPlaced.getFootPose().getX(),
                                             footstepBeingPlaced.getFootPose().getY(),
                                             footstepBeingPlaced.getFootPose().getZ(),
                                             footstepBeingPlaced.getFootPose());

         // When left button clicked and released.
         if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
         {
            forcePlaceFootstep();
         }

         if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            exitPlacement();
         }
      }
   }

   public void placeFootstep()
   {
      // If safe to place footstep
      RDXInteractableFootstep addedStep = footstepPlan.getNextFootstep();
      addedStep.copyFrom(footstepBeingPlaced);
      // Switch sides
      currentFootStepSide = currentFootStepSide.getOppositeSide();
      createNewFootstep(currentFootStepSide);
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
      needToInitializePlacementHeight = true;
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

   private final FramePose3D stanceFootPose = new FramePose3D();
   private final PoseReferenceFrame stanceFootFrame = new PoseReferenceFrame("Latest Stance Foot Frame in Plan", ReferenceFrame.getWorldFrame());

   private final FramePose3D constraintFramePose = new FramePose3D();
   private final PoseReferenceFrame constraintFrame = new PoseReferenceFrame("Latest Control Frame in Plan", ReferenceFrame.getWorldFrame());

   private void applyReachabilityConstraintToStep(FramePose3DReadOnly poseToSet)
   {
      poseToSet.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      RobotSide swingSide = footstepBeingPlaced.getFootstepSide();

      if (footstepPlan.getLastFootstep() != null)
      {
         boolean stanceFootPoseHasBeenSet = false;

         // Make sure stance foot is set from latest footstep in plan that is opposite foot of swing foot
         for (int i = 0 ; i < footstepPlan.getNumberOfFootsteps(); i++)
            if (swingSide != footstepPlan.getFootsteps().get(i).getFootstepSide())
            {
               stanceFootPose.setMatchingFrame(footstepPlan.getFootsteps().get(i).getFootPose());
               stanceFootPoseHasBeenSet = true;
            }

         // If all footsteps in plan are with the same foot, set stanceFootPose to robot's current stance foot
         if (!stanceFootPoseHasBeenSet)
            stanceFootPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(swingSide.getOppositeSide()));

         stanceFootPose.changeFrame(stanceFootFrame.getParent());
         stanceFootFrame.setPoseAndUpdate(stanceFootPose);

         constraintFramePose.setToZero(stanceFootFrame);
         double offset = swingSide == RobotSide.LEFT ? EllipticalStepPositionLimiter.NOMINAL_STANCE_WIDTH_DEFAULT / 2.0 : -EllipticalStepPositionLimiter.NOMINAL_STANCE_WIDTH_DEFAULT / 2.0;
         constraintFramePose.getPosition().addY(offset);
      }
      else
      {
         stanceFootPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(swingSide.getOppositeSide()));

         constraintFramePose.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
         double yaw = syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(syncedRobot.getReferenceFrames().getCenterOfMassFrame()).getRotation().getYaw();
         constraintFramePose.getOrientation().setToYawOrientation(yaw);
      }

      stanceFootPose.changeFrame(stanceFootFrame.getParent());
      constraintFramePose.changeFrame(constraintFrame.getParent());

      stanceFootFrame.setPoseAndUpdate(stanceFootPose);
      constraintFrame.setPoseAndUpdate(constraintFramePose);

      poseToSet.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      if (stepPositionLimiter.hasValue())
         stepPositionLimiter.get().enforceFootPositionConstraint(poseToSet.getPosition(),
                                                                 footstepBeingPlaced.getFootPose().getPosition(),
                                                                 constraintFrame,
                                                                 stanceFootFrame,
                                                                 swingSide);
   }

   public void setFootstepBeingPlacedPose(FramePose3DReadOnly poseToSet)
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
      currentFootStepSide = footstepBeingPlaced.getFootstepSide();
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

      boolean isReachable = footstepBeingPlaced.getFootPose().getPositionDistance(previousFootstepPose) < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepReach();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() < MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();
      isReachable &= footstepBeingPlaced.getFootPose().getZ() - previousFootstepPose.getZ() > -MAX_DISTANCE_MULTIPLIER * footstepPlannerParameters.getMaxStepZ();

      return isReachable;
   }

   public void clear()
   {
      footstepPlan.clear();
   }

   public void setStepPositionLimiter(EllipticalStepPositionLimiter stepPositionLimiter)
   {
      this.stepPositionLimiter.set(stepPositionLimiter);
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.latestHeightMapData = terrainMapData;
   }
}