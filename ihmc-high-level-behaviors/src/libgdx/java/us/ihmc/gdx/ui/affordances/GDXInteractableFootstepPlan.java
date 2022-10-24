package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class GDXInteractableFootstepPlan implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RecyclingArrayList<GDXInteractableFootstep> footsteps = new RecyclingArrayList<>(this::newPlannedFootstep);
   private GDXInteractableFootstep selectedFootstep;
   private GDXImGuiBasedUI baseUI;
   private CommunicationHelper communicationHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private GDXFootstepChecker stepChecker;
   private GDXTeleoperationParameters teleoperationParameters;

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

      stepChecker = new GDXFootstepChecker(baseUI, syncedRobot, footstepPlannerParameters);
      clear();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (GDXInteractableFootstep singleFootstep : footsteps)
      {
         singleFootstep.calculate3DViewPick(input);

         if (singleFootstep.isHovered())
            selectedFootstep = singleFootstep;
      }
      if (selectedFootstep != null)
      {
         selectedFootstep.calculate3DViewPick(input);
      }
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      stepChecker.setRenderTooltip(false);

      // Call each footstep's process3DViewInput
      for (int i = 0; i < footsteps.size(); i++)
      {
         GDXInteractableFootstep singleFootstep = footsteps.get(i);
         singleFootstep.process3DViewInput(input, false);

         if (singleFootstep.isHovered())
         {
            stepChecker.setReasonFrom(i);
            stepChecker.setRenderTooltip(true);
            stepChecker.makeWarnings();
         }
      }
      if (selectedFootstep != null)
      {
         selectedFootstep.process3DViewInput(input, false);
      }

      if (footsteps.size() > 0)
      {
         stepChecker.getInput(input);
      }

      Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();

      // Check validity of footsteps
      stepChecker.checkValidStepList(footsteps);

      // Get the warnings and flash if the footstep's placement isn't okay
      ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
      for (int i = 0; i < temporaryReasons.size(); i++)
      {
         footsteps.get(i).flashFootstepWhenBadPlacement(temporaryReasons.get(i));
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Footstep plan:");
      ImGui.sameLine();
      if (getFootsteps().size() > 0)
      {
         // TODO: Add checker here. Make it harder to walk or give warning if the checker is failing
         if (ImGui.button(labels.get("Walk")) || ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
         {
            walkFromSteps();
         }
         ImGuiTools.previousWidgetTooltip("Keybind: Space");
         ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Delete Last")) || ImGui.isKeyPressed(ImGuiTools.getDeleteKey()))
      {
         removeLastStep();
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Delete");
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXInteractableFootstep footstep : footsteps)
      {
         footstep.getVirtualRenderables(renderables, pool);
      }
   }

   // NOTE: using RecyclingList to update planned footsteps. Make sure this only gets called when new plan comes in.
   public void updateFromPlan(FootstepPlan footstepPlan)
   {
      for (GDXInteractableFootstep step : footsteps)
      {
         step.getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      }
      footsteps.clear();

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep plannedStep = footstepPlan.getFootstep(i);
         GDXInteractableFootstep addedStep = footsteps.add();
         addedStep.updateFromPlannedStep(baseUI, plannedStep, i);
      }
   }

   public void update()
   {
      // Update footsteps in the list, and the one being placed
      for (int i = 0; i < footsteps.size(); i++)
      {
         footsteps.get(i).update();
      }
      if (selectedFootstep != null)
         selectedFootstep.update();

      stepChecker.update(footsteps);
   }

   public void clear()
   {
      footsteps.clear();
      selectedFootstep = null;
   }

   public void walkFromSteps()
   {
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      for (GDXInteractableFootstep step : footsteps)
      {
         generateFootStepDataMessage(messageList, step);
         messageList.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
         messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
      communicationHelper.publishToController(messageList);

      // Note: set stance and swing as last two steps of the footstepArrayList (if this list is not empty)
      // Note: delete steps in singleFootStepAffordance.

      if (footsteps.size() == 1)
      {
         stepChecker.setStanceStepTransform(footsteps.get(0).getFootTransformInWorld());
         stepChecker.setStanceSide(footsteps.get(0).getFootstepSide());
      }
      else if (footsteps.size() > 1)
      {
         int size = footsteps.size();
         stepChecker.setStanceStepTransform(footsteps.get(size - 1).getFootTransformInWorld());
         stepChecker.setStanceSide(footsteps.get(size - 1).getFootstepSide());
         stepChecker.setSwingStepTransform(footsteps.get(size - 2).getFootTransformInWorld());
         stepChecker.setSwingSide(footsteps.get(size - 2).getFootstepSide());
      }
      stepChecker.clear(footsteps);
      clear();
   }

   private void generateFootStepDataMessage(FootstepDataListMessage messageList, GDXInteractableFootstep step)
   {
      FootstepDataMessage stepMessage = messageList.getFootstepDataList().add();
      stepMessage.setRobotSide(step.getFootstepSide().toByte());
      stepMessage.getLocation().set(new Point3D(step.getSelectablePose3DGizmo().getPoseGizmo().getPose().getPosition()));
      stepMessage.getOrientation().set(step.getPose().getOrientation());
      stepMessage.setSwingDuration(teleoperationParameters.getSwingTime());
      stepMessage.setTransferDuration(teleoperationParameters.getTransferTime());
   }

   public RecyclingArrayList<GDXInteractableFootstep> getFootsteps()
   {
      return footsteps;
   }

   public void removeLastStep()
   {
      // TODO: This is not necessary but, safe for now
      selectedFootstep = null;
      if (!footsteps.isEmpty())
      {
         footsteps.remove(footsteps.size() - 1);
      }
   }

   /**
    * Gets the transform either from the footstep list, or from the synced robot.
    * Never gets the transform from the footstep currently being placed.
    */
   public RigidBodyTransform getLastFootstepTransform(RobotSide robotSide)
   {
      if (footsteps.size() > 0)
      {
         return getLastFootstep().getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent();
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }
   }

   public GDXInteractableFootstep getLastFootstep()
   {
      if (footsteps.size() > 0)
      {
         return footsteps.get(footsteps.size() - 1);
      }
      return null;
   }

   private GDXInteractableFootstep newPlannedFootstep()
   {
      return new GDXInteractableFootstep(baseUI, RobotSide.LEFT, 0);
   }

   public GDXFootstepChecker getStepChecker()
   {
      return stepChecker;
   }
}
