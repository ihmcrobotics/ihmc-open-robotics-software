package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class GDXInteractableFootstepPlan implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final Pose3D goalPoseForReading = new Pose3D();
   private final RecyclingArrayList<ImGuiGDXPlannedFootstep> footstepArrayList = new RecyclingArrayList<>(this::newPlannedFootstep);
   private ImGuiGDXPlannedFootstep footstepBeingPlaced;
   private int footstepIndex = -1;
   private GDXImGuiBasedUI baseUI;
   private CommunicationHelper communicationHelper;
   private RobotSide currentFootStepSide;
   private ROS2SyncedRobotModel syncedRobot;
   private ImGuiGDXPlannedFootstepChecker stepChecker;
   private ImGui3DViewInput latestInput;
   private GDX3DPanel primary3DPanel;
   private GDXTeleoperationParameters teleoperationParameters;
   private ImGuiGDXPlannedFootstep stepBeingModified = null;

   FramePose3D tempFramePose = new FramePose3D();

   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private GDXIconTexture feetIcon;

   public void create(GDXImGuiBasedUI baseUI,
                      CommunicationHelper communicationHelper,
                      ROS2SyncedRobotModel syncedRobot,
                      GDXTeleoperationParameters teleoperationParameters,
                      FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.teleoperationParameters = teleoperationParameters;
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();

      stepChecker = new ImGuiGDXPlannedFootstepChecker(baseUI, communicationHelper, syncedRobot, footstepPlannerParameters);
      clear();

      feetIcon = new GDXIconTexture(iconDirectory.file("feet.png"));
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (ImGuiGDXPlannedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.calculate3DViewPick(input);

         if (singleFootstep.isPickSelected())
            stepBeingModified = singleFootstep;
      }
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.calculate3DViewPick(input);
      }
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;
      boolean anyFootstepIsSelected = false;

      // Call each footstep's process3DViewInput
      for (ImGuiGDXPlannedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.process3DViewInput(input);
      }
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.process3DViewInput(input);
      }

      if (footstepArrayList.size() > 0)
      {
         stepChecker.getInput(input);
      }

      Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();

      // Check validity of footsteps
      stepChecker.checkValidStepList(footstepArrayList);
      if (isCurrentlyPlacingFootstep())
      {
         RigidBodyTransform candidateStepTransform = new RigidBodyTransform();
         candidateStepTransform.getTranslation().set(pickPointInWorld);
         candidateStepTransform.getRotation().setToYawOrientation(getFootstepBeingPlacedOrLastFootstepPlaced().getYaw());

         stepChecker.checkValidSingleStep(footstepArrayList, candidateStepTransform, currentFootStepSide, footstepArrayList.size());
      }

      // Get the warnings and flash if the footstep's placement isn't okay
      ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
      if (temporaryReasons.size() > 0)
      {
         for (int i = 0; i < temporaryReasons.size(); i++)
         {
            if (footstepArrayList.size() > i)
               footstepArrayList.get(i).flashFootstepWhenBadPlacement(temporaryReasons.get(i));
         }
         if (footstepBeingPlaced != null)
            footstepBeingPlaced.flashFootstepWhenBadPlacement(temporaryReasons.get(temporaryReasons.size() - 1));
      }

      anyFootstepIsSelected = isAnyFootstepSelected();

      if (anyFootstepIsSelected)
      {
         stepChecker.setRenderTooltip(true);
         stepChecker.makeWarnings();
      }
      else
      {
         stepChecker.setRenderTooltip(false);
      }
   }

   private boolean isAnyFootstepSelected()
   {
      // Generate the warning messages on the tooltips
      boolean anyFootstepIsSelected = false;
      if (footstepBeingPlaced == null)
      {
         if (footstepArrayList.size() > 0)
         {
            for (int i = 0; i < footstepArrayList.size(); ++i)
            {
               if (footstepArrayList.get(i).isPickSelected())
               {
                  stepChecker.setReasonFrom(i);
                  anyFootstepIsSelected = true;
                  break;
               }
            }
         }
      }
      else
      {
         anyFootstepIsSelected = true;
      }
      return anyFootstepIsSelected;
   }

   private void placeFootstep()
   {
      footstepIndex++;
      footstepArrayList.add(footstepBeingPlaced);

      //Switch sides
      currentFootStepSide = currentFootStepSide.getOppositeSide();
      createNewFootStep(currentFootStepSide);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
      {
         if (getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
      }

      if (ImGui.isKeyPressed(ImGuiTools.getDeleteKey()))
      {
         removeFootStep();
      }

      // TODO: experimental (interactable footsteps from control ring paths)
      if (imgui.ImGui.isKeyPressed('O'))
      {
         if (getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ImGuiGDXPlannedFootstep step : footstepArrayList)
      {
         step.getVirtualRenderables(renderables, pool);
      }
   }

   // TODO: using recycling list to update planned footsteps. Make sure this only gets called when new plan comes in.
   public void updateFromPlan(FootstepPlan footstepPlan)
   {
      for (ImGuiGDXPlannedFootstep step : footstepArrayList)
      {
         step.getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      }
      footstepArrayList.clear();

      ArrayList<PlannedFootstep> plannedSteps = footstepPlan.getFootsteps();
      for (int i = 0; i < plannedSteps.size(); ++i)
      {
         PlannedFootstep plannedStep = plannedSteps.get(i);
         ImGuiGDXPlannedFootstep addedStep = footstepArrayList.add();
         addedStep.updateFromPlannedStep(baseUI, plannedStep, i);
      }
   }

   public void update()
   {
      // Update footsteps in the list, and the one being placed
      for (int i = 0; i < footstepArrayList.size(); i++)
      {
         footstepArrayList.get(i).update();
      }
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.update();

      stepChecker.update(footstepArrayList);
   }

   public void clear()
   {
      // Remove all footsteps
      while (footstepArrayList.size() > 0)
      {
         removeFootStep();
      }
      stepBeingModified = null;
      footstepIndex = -1;
   }

   public void walkFromSteps()
   {
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      for (ImGuiGDXPlannedFootstep step : footstepArrayList)
      {
         generateFootStepDataMessage(messageList, step);
         messageList.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
      communicationHelper.publishToController(messageList);

      // Note: set stance and swing as last two steps of the footstepArrayList (if this list is not empty)
      // Note: delete steps in singleFootStepAffordance.

      if (footstepArrayList.size() == 1)
      {
         stepChecker.setStanceStepTransform(footstepArrayList.get(0).getFootTransformInWorld());
         stepChecker.setStanceSide(footstepArrayList.get(0).getFootstepSide());
      }
      else if (footstepArrayList.size() > 1)
      {
         int size = footstepArrayList.size();
         stepChecker.setStanceStepTransform(footstepArrayList.get(size - 1).getFootTransformInWorld());
         stepChecker.setStanceSide(footstepArrayList.get(size - 1).getFootstepSide());
         stepChecker.setSwingStepTransform(footstepArrayList.get(size - 2).getFootTransformInWorld());
         stepChecker.setSwingSide(footstepArrayList.get(size - 2).getFootstepSide());
      }
      stepChecker.clear(footstepArrayList);
      clear();
   }

   private void generateFootStepDataMessage(FootstepDataListMessage messageList, ImGuiGDXPlannedFootstep step)
   {
      FootstepDataMessage stepMessage = messageList.getFootstepDataList().add();
      stepMessage.setRobotSide(step.getFootstepSide().toByte());
      stepMessage.getLocation().set(new Point3D(step.getSelectablePose3DGizmo().getPoseGizmo().getPose().getPosition()));
      stepMessage.getOrientation().set(step.getPose().getOrientation());
      stepMessage.setSwingDuration(teleoperationParameters.getSwingTime());
      stepMessage.setTransferDuration(teleoperationParameters.getTransferTime());
   }

   public RecyclingArrayList<ImGuiGDXPlannedFootstep> getFootstepArrayList()
   {
      return footstepArrayList;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      if (footstepBeingPlaced != null)
      {
         removeFootStep();
      }

      RigidBodyTransform latestFootstepTransform = getLatestPlacedFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();
      footstepBeingPlaced = new ImGuiGDXPlannedFootstep(baseUI, footstepSide, footstepIndex);
      currentFootStepSide = footstepSide;

      // Note: set the yaw of the new footstep to the yaw of the previous footstep
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().set(new RotationMatrix(latestFootstepYaw, 0.0, 0.0));
      tempFramePose.get(footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
      footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();
   }

   public void removeFootStep()
   {
      if (footstepBeingPlaced != null || footstepArrayList.size() > 0)
      {
         if (getFootstepBeingPlacedOrLastFootstepPlaced() != null && getFootstepBeingPlacedOrLastFootstepPlaced().getFootstepModelInstance() != null)
         {
            getFootstepBeingPlacedOrLastFootstepPlaced().getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
         }

         if (footstepBeingPlaced == null)
         {
            footstepIndex--;
            footstepArrayList.remove(footstepArrayList.size() - 1);
         }
         else
         {
            footstepBeingPlaced = null;
         }
      }
   }

   /**
    * Gets the transform either from the footstep list, or from the synced robot.
    * Never gets the transform from the footstep currently being placed.
    */
   public RigidBodyTransform getLatestPlacedFootstepTransform(RobotSide robotSide)
   {
      if (footstepArrayList.size() > 0)
      {
         return footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent();
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }
   }

   /**
    * Returns future footstep currently being placed. If you are not placing a footstep currently, it will return last footstep from list.
    * Does NOT return footsteps that you already walked on
    */
   public ImGuiGDXPlannedFootstep getFootstepBeingPlacedOrLastFootstepPlaced()
   {
      if (footstepBeingPlaced != null)
      {
         return footstepBeingPlaced;
      }
      else if (footstepArrayList.size() > 0)
      {
         return footstepArrayList.get(footstepArrayList.size() - 1);
      }
      return null;
   }

   public boolean isCurrentlyPlacingFootstep()
   {
      if (footstepBeingPlaced == null)
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   private ImGuiGDXPlannedFootstep newPlannedFootstep()
   {
      return new ImGuiGDXPlannedFootstep(baseUI, RobotSide.LEFT, 0);
   }
}
