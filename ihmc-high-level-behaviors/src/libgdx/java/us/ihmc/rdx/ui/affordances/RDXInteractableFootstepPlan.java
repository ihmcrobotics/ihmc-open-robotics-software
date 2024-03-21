package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
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
   private final ControllerStatusTracker controllerStatusTracker;
   private final RecyclingArrayList<RDXInteractableFootstep> footsteps = new RecyclingArrayList<>(this::newPlannedFootstep);
   private RDXInteractableFootstep selectedFootstep;
   private RDXBaseUI baseUI;
   private CommunicationHelper communicationHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private RDXFootstepChecker stepChecker;
   private RDXSwingPlanningModule swingPlanningModule;
   private SideDependentList<ConvexPolygon2D> defaultPolygons;
   private RDXLocomotionParameters locomotionParameters;
   private SwingPlannerParametersBasics swingFootPlannerParameters;

   private final AtomicReference<HeightMapMessage> heightMapDataReference = new AtomicReference<>();

   private int previousPlanLength;
   private boolean wasPlanUpdated = false;

   public RDXInteractableFootstepPlan(ControllerStatusTracker controllerStatusTracker)
   {
      this.controllerStatusTracker = controllerStatusTracker;
   }

   public void create(RDXBaseUI baseUI,
                      CommunicationHelper communicationHelper,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXLocomotionParameters locomotionParameters,
                      FootstepPlannerParametersReadOnly footstepPlannerParameters,
                      FootstepPlannerParametersReadOnly turnWalkTurnFootstepPlannerParameters,
                      SwingPlannerParametersBasics swingFootPlannerParameters)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      this.locomotionParameters = locomotionParameters;
      this.swingFootPlannerParameters = swingFootPlannerParameters;

      defaultPolygons = FootstepPlanningModuleLauncher.createFootPolygons(communicationHelper.getRobotModel());
      stepChecker = new RDXFootstepChecker(baseUI,
                                           syncedRobot,
                                           controllerStatusTracker,
                                           defaultPolygons,
                                           locomotionParameters,
                                           footstepPlannerParameters,
                                           turnWalkTurnFootstepPlannerParameters);
      swingPlanningModule = new RDXSwingPlanningModule(syncedRobot,
                                                       locomotionParameters,
                                                       footstepPlannerParameters,
                                                       turnWalkTurnFootstepPlannerParameters,
                                                       communicationHelper.getRobotModel().getSwingPlannerParameters(),
                                                       communicationHelper.getRobotModel().getWalkingControllerParameters(),
                                                       defaultPolygons);
      clear();
   }

   public void setHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      heightMapDataReference.set(heightMapMessage);
      if (swingPlanningModule != null)
         swingPlanningModule.setHeightMapData(heightMapMessage);
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RDXInteractableFootstep footstep : footsteps)
      {
         footstep.calculateVRPick(vrContext);
      }
      if (selectedFootstep != null)
      {
         selectedFootstep.calculateVRPick(vrContext);
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (RDXInteractableFootstep footstep : footsteps)
      {
         footstep.processVRInput(vrContext);
      }

      if (selectedFootstep != null)
      {
         selectedFootstep.processVRInput(vrContext);
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXInteractableFootstep footstep : footsteps)
      {
         footstep.calculate3DViewPick(input);

         if (footstep.isMouseHovering())
            selectedFootstep = footstep;
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
         RDXInteractableFootstep footstep = footsteps.get(i);
         footstep.process3DViewInput(input, false);

         if (footstep.isMouseHovering())
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

      if (wasPlanUpdated && locomotionParameters.getReplanSwingTrajectoryOnChange() && !swingPlanningModule.getIsCurrentlyPlanning())
      {
         HeightMapMessage heightMapMessage = heightMapDataReference.getAndSet(null);
         if (heightMapMessage != null)
            swingPlanningModule.setHeightMapData(heightMapMessage);

         swingPlanningModule.setSwingPlannerParameters(swingFootPlannerParameters);
         swingPlanningModule.updateAysnc(footsteps, SwingPlannerType.MULTI_WAYPOINT_POSITION);

         wasPlanUpdated = false;
      }

      stepChecker.update(footsteps);

      // Get the warnings and flash if the footstep's placement isn't okay
      ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
      for (int i = 0; i < temporaryReasons.size(); i++)
      {
         footsteps.get(i).flashFootstepWhenBadPlacement(temporaryReasons.get(i));
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
      stepChecker.clear();
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

      RDXBaseUI.pushNotification("Commanding %d footsteps...".formatted(messageList.getFootstepDataList().size()));
      communicationHelper.publishToController(messageList);

      clear();
   }

   public RDXInteractableFootstep getNextFootstep()
   {
      RDXInteractableFootstep nextFootstep = footsteps.add();
      nextFootstep.reset();
      return nextFootstep;
   }

   public RDXInteractableFootstep getLastFootstep()
   {
      if (footsteps.size() > 0)
      {
         return footsteps.get(footsteps.size() - 1);
      }
      return null;
   }

   /**
    * Gets the transform of the last footstep, first checking the current interactable footstep list, then checking the queued controller footsteps,
    * and if there are no footsteps in either of those. Use the feet of the synced robot to get the transform
    * Never gets the transform from the footstep currently being placed.
    */
   public RigidBodyTransformReadOnly getLastFootstepTransform(RobotSide robotSide)
   {
      if (!footsteps.isEmpty())
      {
         return getLastFootstep().getFootPose();
      }
      else if (controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps() > 0)
      {
         return controllerStatusTracker.getFootstepTracker().getLastFootstepQueuedOnOppositeSide(robotSide.getOppositeSide());
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }
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
