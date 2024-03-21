package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

/**
 * Tells the operator if a manually placed footstep is reasonable in realtime.
 * Footsteps will flash if the footstep is unreasonable, and when the mouse hovers over the footstep a reason is displayed informing the user why that footstep
 * is unreasonable.
 */
public class RDXFootstepChecker
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private final RDXLocomotionParameters locomotionParameters;
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final FootstepPlannerParametersReadOnly turnWalkTurnFootstepPlannerParameters;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepSnapAndWiggler snapper;
   private final FootstepPoseHeuristicChecker stepChecker;
   private final ArrayList<BipedalFootstepPlannerNodeRejectionReason> reasons = new ArrayList<>();
   private final RDX3DPanelTooltip tooltip;

   private BipedalFootstepPlannerNodeRejectionReason reason = null;
   private String text = null;
   private boolean renderTooltip = false;

   public RDXFootstepChecker(RDXBaseUI baseUI,
                             ROS2SyncedRobotModel syncedRobot,
                             ControllerStatusTracker controllerStatusTracker,
                             SideDependentList<ConvexPolygon2D> footPolygons,
                             RDXLocomotionParameters locomotionParameters,
                             FootstepPlannerParametersReadOnly footstepPlannerParameters,
                             FootstepPlannerParametersReadOnly turnWalkTurnFootstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.locomotionParameters = locomotionParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.turnWalkTurnFootstepPlannerParameters = turnWalkTurnFootstepPlannerParameters;
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::renderTooltips);
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      tooltip = new RDX3DPanelTooltip(baseUI.getPrimary3DPanel());
      snapper = new FootstepSnapAndWiggler(footPolygons, footstepPlannerParameters, environmentHandler);
      stepChecker = new FootstepPoseHeuristicChecker(footstepPlannerParameters, snapper, registry);
   }

   public void getInput(ImGui3DViewInput input)
   {
      tooltip.setInput(input);
   }

   private void renderTooltips()
   {
      if (renderTooltip)
      {
         tooltip.render(text);
      }
   }

   public void checkValidStepList(RecyclingArrayList<RDXInteractableFootstep> stepList)
   {
      reasons.clear();

      // Iterate through the footstep list and check the validity for all footsteps.
      for (int i = 0; i < stepList.size(); ++i)
      {
         checkValidSingleStep(stepList, stepList.get(i).getFootPose(), stepList.get(i).getFootstepSide(), i);
      }
   }

   public void checkValidSingleStep(RecyclingArrayList<RDXInteractableFootstep> stepList,
                                    FramePose3DReadOnly candidateFootstepPose,
                                    RobotSide candidateStepSide,
                                    int indexOfFootBeingChecked /* list.size() if not placed yet*/)
   {
      FramePose3DReadOnly previousFootstepOnOtherSide = getPreviousFootstepOnOppositeSide(stepList, indexOfFootBeingChecked, candidateStepSide);
      FramePose3DReadOnly previousFootstepOnSameSide = getPreviousFootstepOnOppositeSide(stepList,
                                                                                         indexOfFootBeingChecked,
                                                                                         candidateStepSide.getOppositeSide());
      if (locomotionParameters.getPerformAStarSearch())
      {
         snapper.setParameters(footstepPlannerParameters);
         stepChecker.setParameters(footstepPlannerParameters);
      }
      else
      {
         snapper.setParameters(turnWalkTurnFootstepPlannerParameters);
         stepChecker.setParameters(turnWalkTurnFootstepPlannerParameters);
      }

      reason = stepChecker.checkValidity(candidateStepSide, candidateFootstepPose, previousFootstepOnOtherSide, previousFootstepOnSameSide);

      reasons.add(reason);
   }

   /**
    * Returns the previous footstep on the opposite side of the new footstep side if it exists, otherwise set it to the current robot foot
    * First check against footsteps that have been placed but are not sent to the controller
    * Second check against footsteps that are in the controller
    * Lastly if those don't have footsteps, default to comparing against the synced robot feet
    */
   public FramePose3DReadOnly getPreviousFootstepOnOppositeSide(RecyclingArrayList<RDXInteractableFootstep> stepList,
                                                                 int currentIndex,
                                                                 RobotSide candidateFootstepSide)
   {
      FramePose3D previousFootstepPose = new FramePose3D();

      // Need to subtract one if our current index is for the current step that is being placed cause its not in the list
      if (currentIndex == stepList.size())
         currentIndex = currentIndex - 1;

      // Moved the index of the list to the last step on the other side
      int i = currentIndex;
      while (i >= 0 && stepList.get(i).getFootstepSide() == candidateFootstepSide)
         --i;

      if (i >= 0)
      {
         previousFootstepPose.setIncludingFrame(stepList.get(i).getFootPose());
      }
      else if (controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps() > 0)
      {
         previousFootstepPose.set(controllerStatusTracker.getFootstepTracker().getLastFootstepQueuedOnOppositeSide(candidateFootstepSide));
      }
      else
      {
         previousFootstepPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(candidateFootstepSide.getOppositeSide()));
      }

      return previousFootstepPose;
   }

   public void makeWarnings()
   {
      if (reason != null)
      {
         text = "Rejected for %s.".formatted(reason.name());
      }
      else
      {
         text = "Passes checks.";
      }
   }

   public void clear()
   {
      reasons.clear();
   }

   public void setReasonFrom(int i)
   {
      reason = reasons.get(i);
   }

   public ArrayList<BipedalFootstepPlannerNodeRejectionReason> getReasons()
   {
      return reasons;
   }

   public void setRenderTooltip(boolean renderTooltip)
   {
      this.renderTooltip = renderTooltip;
   }

   public void update(RecyclingArrayList<RDXInteractableFootstep> footstepArrayList)
   {
      checkValidStepList(footstepArrayList);
   }
}
