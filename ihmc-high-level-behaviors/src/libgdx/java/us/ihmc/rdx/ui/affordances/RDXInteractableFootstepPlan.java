package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.ImGui;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class RDXInteractableFootstepPlan implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RecyclingArrayList<RDXInteractableFootstep> footsteps = new RecyclingArrayList<>(this::newPlannedFootstep);
   private RDXInteractableFootstep selectedFootstep;
   private RDXBaseUI baseUI;
   private CommunicationHelper communicationHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private RDXFootstepChecker stepChecker;
   private RDXSwingPlanningModule swingPlanningModule;
   private RDXTeleoperationParameters teleoperationParameters;

   public void create(RDXBaseUI baseUI,
                      CommunicationHelper communicationHelper,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXTeleoperationParameters teleoperationParameters,
                      FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.teleoperationParameters = teleoperationParameters;
      this.syncedRobot = syncedRobot;

      SideDependentList<ConvexPolygon2D> footPolygons = FootstepPlanningModuleLauncher.createFootPolygons(communicationHelper.getRobotModel());
      stepChecker = new RDXFootstepChecker(baseUI, syncedRobot, footPolygons, footstepPlannerParameters);
      swingPlanningModule = new RDXSwingPlanningModule(syncedRobot,
                                                       footstepPlannerParameters,
                                                       communicationHelper.getRobotModel().getSwingPlannerParameters(),
                                                       communicationHelper.getRobotModel().getWalkingControllerParameters(),
                                                       footPolygons);
      clear();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXInteractableFootstep singleFootstep : footsteps)
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

   public void setPlanarRegionsList(PlanarRegionsListMessage planarRegionsList)
   {
      swingPlanningModule.setPlanarRegionList(planarRegionsList);
   }

   public void setHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      swingPlanningModule.setHeightMapData(heightMapMessage);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      stepChecker.setRenderTooltip(false);

      // Call each footstep's process3DViewInput
      for (int i = 0; i < footsteps.size(); i++)
      {
         RDXInteractableFootstep singleFootstep = footsteps.get(i);
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
      for (RDXInteractableFootstep footstep : footsteps)
      {
         footstep.getVirtualRenderables(renderables, pool);
      }
   }

   // NOTE: using RecyclingList to update planned footsteps. Make sure this only gets called when new plan comes in.
   public void updateFromPlan(FootstepPlan footstepPlan)
   {
      for (RDXInteractableFootstep step : footsteps)
      {
         step.getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      }
      footsteps.clear();

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep plannedStep = footstepPlan.getFootstep(i);
         RDXInteractableFootstep addedStep = footsteps.add();
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
      swingPlanningModule.update(footsteps, SwingPlannerType.MULTI_WAYPOINT_POSITION);
   }

   public void clear()
   {
      footsteps.clear();
      selectedFootstep = null;
   }

   public void walkFromSteps()
   {
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      for (RDXInteractableFootstep step : footsteps)
      {
         messageList.getFootstepDataList().add().set(step.getPlannedFootstep().getAsMessage());
         messageList.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
         messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
      messageList.setOffsetFootstepsHeightWithExecutionError(true);
      messageList.setDefaultSwingDuration(teleoperationParameters.getSwingTime());
      messageList.setDefaultTransferDuration(teleoperationParameters.getTransferTime());

      communicationHelper.publishToController(messageList);

      // Note: set stance and swing as last two steps of the footstepArrayList (if this list is not empty)
      // Note: delete steps in singleFootStepAffordance.

      if (footsteps.size() == 1)
      {
         stepChecker.setPreviousStepPose(footsteps.get(0).getFootPose());
         stepChecker.setStanceSide(footsteps.get(0).getFootstepSide());
      }
      else if (footsteps.size() > 1)
      {
         int size = footsteps.size();
         stepChecker.setPreviousStepPose(footsteps.get(size - 1).getFootPose());
         stepChecker.setStanceSide(footsteps.get(size - 1).getFootstepSide());
         stepChecker.setSwingStepPose(footsteps.get(size - 2).getFootPose());
         stepChecker.setSwingSide(footsteps.get(size - 2).getFootstepSide());
      }
      stepChecker.clear(footsteps);
      clear();
   }

   public RecyclingArrayList<RDXInteractableFootstep> getFootsteps()
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
   public RigidBodyTransformReadOnly getLastFootstepTransform(RobotSide robotSide)
   {
      if (footsteps.size() > 0)
      {
         return getLastFootstep().getFootPose();
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }
   }

   public RDXInteractableFootstep getLastFootstep()
   {
      if (footsteps.size() > 0)
      {
         return footsteps.get(footsteps.size() - 1);
      }
      return null;
   }

   private RDXInteractableFootstep newPlannedFootstep()
   {
      return new RDXInteractableFootstep(baseUI, RobotSide.LEFT, 0);
   }

   public RDXFootstepChecker getStepChecker()
   {
      return stepChecker;
   }

   public void destroy()
   {
      swingPlanningModule.destroy();
   }
}
