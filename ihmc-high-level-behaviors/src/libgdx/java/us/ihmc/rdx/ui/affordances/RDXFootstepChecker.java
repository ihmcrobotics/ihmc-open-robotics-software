package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
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
   private final RDX3DPanel primary3DPanel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepSnapAndWiggler snapper;
   private final FootstepPoseHeuristicChecker stepChecker;
   private final ArrayList<BipedalFootstepPlannerNodeRejectionReason> reasons = new ArrayList<>();
   private final RDX3DPanelTooltip tooltip;

   private final SideDependentList<RigidBodyTransformReadOnly> syncedSolePoses = new SideDependentList<>();
   private BipedalFootstepPlannerNodeRejectionReason reason = null;
   private String text = null;
   private boolean renderTooltip = false;

   public RDXFootstepChecker(RDXBaseUI baseUI,
                             ROS2SyncedRobotModel syncedRobot,
                             SideDependentList<ConvexPolygon2D> footPolygons,
                             FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();
      tooltip = new RDX3DPanelTooltip(primary3DPanel);
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler(footPolygons);
      snapper = new FootstepSnapAndWiggler(footPolygons, footstepPlannerParameters, environmentHandler);
      stepChecker = new FootstepPoseHeuristicChecker(footstepPlannerParameters, snapper, registry);
      setInitialFeet();
   }

   public void setInitialFeet()
   {
      for (RobotSide side : RobotSide.values)
      {
         syncedSolePoses.put(side, syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToRoot());
      }
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

   // TODO: This should update candidate, stance, and swing in the ImGuiGDXManualFootstepPlacement,
   //  updates RejectionReason, and generate warning message in the UI screen.
   public void checkValidStepList(RecyclingArrayList<RDXInteractableFootstep> stepList)
   {
      reasons.clear();
      setInitialFeet();
      // iterate through the list ( + current initial stance and swing) and check validity for all.
      for (int i = 0; i < stepList.size(); ++i)
      {
         checkValidSingleStep(stepList, stepList.get(i).getFootPose(), stepList.get(i).getFootstepSide(), i);
      }
   }

   // Check the validity of a single step
   public void checkValidSingleStep(RecyclingArrayList<RDXInteractableFootstep> stepList,
                                    FramePose3DReadOnly candidateFootstepPose,
                                    RobotSide candidateStepSide,
                                    int indexOfFootBeingChecked /* list.size() if not placed yet*/)
   {
      if (indexOfFootBeingChecked == 0) // No footsteps have been placed, check against robot feet
      {
         reason = stepChecker.checkValidity(candidateStepSide,
                                            candidateFootstepPose,
                                            syncedSolePoses.get(candidateStepSide.getOppositeSide()),
                                            syncedSolePoses.get(candidateStepSide));
      }
      else if (indexOfFootBeingChecked == 1) // One footstep has been placed, check against opposite robot foot
      {
         FramePose3DReadOnly previousFootstepOnOtherSide = getPreviousFootstepOnOppositeSide(stepList, indexOfFootBeingChecked, candidateStepSide);
         reason = stepChecker.checkValidity(candidateStepSide, candidateFootstepPose, previousFootstepOnOtherSide, syncedSolePoses.get(candidateStepSide));
      }
      else // Multiple footsteps have been placed, compare to previous footstep on the other side
      {
         FramePose3DReadOnly previousFootstepOnSameSide = getPreviousFootstepOnOppositeSide(stepList,
                                                                                            indexOfFootBeingChecked,
                                                                                            candidateStepSide.getOppositeSide());
         FramePose3DReadOnly previousFootstepOnOtherSide = getPreviousFootstepOnOppositeSide(stepList, indexOfFootBeingChecked, candidateStepSide);

         reason = stepChecker.checkValidity(candidateStepSide,
                                            candidateFootstepPose,
                                            previousFootstepOnOtherSide,
                                            previousFootstepOnSameSide);
      }

      reasons.add(reason);
   }

   /**
    * Sets the previousFootstepPose to the footstep on the opposite side of the candidateFootstepSide if it exists, otherwise set it to the current robot foot
    */
   public FramePose3DReadOnly getPreviousFootstepOnOppositeSide(RecyclingArrayList<RDXInteractableFootstep> stepList, int currentIndex, RobotSide candidateFootstepSide)
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

   public RDX3DPanel getPrimary3DPanel()
   {
      return primary3DPanel;
   }

   public void update(RecyclingArrayList<RDXInteractableFootstep> footstepArrayList)
   {
      checkValidStepList(footstepArrayList);
   }
}
