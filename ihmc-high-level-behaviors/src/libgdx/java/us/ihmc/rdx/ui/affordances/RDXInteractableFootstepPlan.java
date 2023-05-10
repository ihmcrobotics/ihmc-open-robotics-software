package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicReference;

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
   private SideDependentList<ConvexPolygon2D> defaultPolygons;
   private RDXLocomotionParameters locomotionParameters;

   private final AtomicReference<HeightMapMessage> heightMapReference = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListReference = new AtomicReference<>();
   private final AtomicReference<SwingPlannerParametersReadOnly> swingPlannerParametersReference = new AtomicReference<>();

   private int previousPlanLength;
   private boolean wasPlanUpdated = false;

   public void create(RDXBaseUI baseUI,
                      CommunicationHelper communicationHelper,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXLocomotionParameters locomotionParameters,
                      FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.locomotionParameters = locomotionParameters;
      this.syncedRobot = syncedRobot;

      defaultPolygons = FootstepPlanningModuleLauncher.createFootPolygons(communicationHelper.getRobotModel());
      stepChecker = new RDXFootstepChecker(baseUI, syncedRobot, defaultPolygons, footstepPlannerParameters);
      swingPlanningModule = new RDXSwingPlanningModule(syncedRobot,
                                                       footstepPlannerParameters,
                                                       communicationHelper.getRobotModel().getSwingPlannerParameters(),
                                                       communicationHelper.getRobotModel().getWalkingControllerParameters(),
                                                       defaultPolygons);
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
      planarRegionsListReference.set(planarRegionsList);
      if (swingPlanningModule != null)
         swingPlanningModule.setPlanarRegionList(planarRegionsList);
   }

   public void setHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      heightMapReference.set(heightMapMessage);
      if (swingPlanningModule != null)
         swingPlanningModule.setHeightMapData(heightMapMessage);
   }

   public void setSwingPlannerParameters(SwingPlannerParametersReadOnly swingPlannarParameters)
   {
      swingPlannerParametersReference.set(swingPlannarParameters);
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
      if (footsteps.size() > 0)
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
   public void updateFromPlan(FootstepPlan footstepPlan, List< EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories)
   {
      for (RDXInteractableFootstep step : footsteps)
      {
         step.reset();
      }
      footsteps.clear();

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep plannedStep = footstepPlan.getFootstep(i);
         RDXInteractableFootstep addedStep = footsteps.add();
         addedStep.reset();
         EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory;
         if (swingTrajectories == null)
         {
            swingTrajectory = null;
         }
         else
         {
            if (i < swingTrajectories.size())
               swingTrajectory = swingTrajectories.get(i);
            else
               swingTrajectory = null;
         }
         addedStep.updateFromPlannedStep(baseUI, plannedStep, swingTrajectory, i);
      }

      wasPlanUpdated = true;
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

      // check if the plan grew in length
      wasPlanUpdated |= previousPlanLength < footsteps.size();
      wasPlanUpdated |= pollIfAnyStepWasUpdated();
      previousPlanLength = footsteps.size();

      stepChecker.update(footsteps);
      if (wasPlanUpdated && locomotionParameters.getReplanSwingTrajectoryOnChange() && !swingPlanningModule.getIsCurrentlyPlanning())
      {
         PlanarRegionsListMessage planarRegionsList = planarRegionsListReference.getAndSet(null);
         if (planarRegionsList != null)
            swingPlanningModule.setPlanarRegionList(planarRegionsList);
         HeightMapMessage heightMapMessage = heightMapReference.getAndSet(null);
         if (heightMapMessage != null)
            swingPlanningModule.setHeightMapData(heightMapMessage);
         SwingPlannerParametersReadOnly swingPlannerParameters = swingPlannerParametersReference.getAndSet(null);
         if (swingPlannerParameters != null)
         {
            swingPlanningModule.setSwingPlannerParameters(swingPlannerParameters);
            swingPlanningModule.updateAysnc(footsteps, SwingPlannerType.MULTI_WAYPOINT_POSITION);
         }

         wasPlanUpdated = false;
      }
   }

   private boolean pollIfAnyStepWasUpdated()
   {
      boolean wasUpdated = false;
      for (int i = 0; i < footsteps.size(); i++)
         wasUpdated |= footsteps.get(i).pollWasPoseUpdated();
      return wasUpdated;
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
      }
      // TODO figure out some better logic here. For example, when footstep planning from the current pose, or using the control ring, this is probably pretty
      // TODO dangerous. However when manually placing footsteps, this is great.
      messageList.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      messageList.setOffsetFootstepsHeightWithExecutionError(true);
      messageList.setDefaultSwingDuration(locomotionParameters.getSwingTime());
      messageList.setDefaultTransferDuration(locomotionParameters.getTransferTime());
      messageList.setAreFootstepsAdjustable(locomotionParameters.getAreFootstepsAdjustable());

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

   public RDXInteractableFootstep getNextFootstep()
   {
      RDXInteractableFootstep nextFootstep = footsteps.add();
      nextFootstep.reset();
      return nextFootstep;
   }

   public int getNumberOfFootsteps()
   {
      return footsteps.size();
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
      wasPlanUpdated = true;
      return new RDXInteractableFootstep(baseUI, RobotSide.LEFT, 0, defaultPolygons);
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
