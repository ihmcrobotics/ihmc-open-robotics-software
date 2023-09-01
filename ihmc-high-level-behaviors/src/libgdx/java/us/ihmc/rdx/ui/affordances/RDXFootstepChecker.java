package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.SwingPlanningModule;
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
 * Helps the operator confirm that manually placed footsteps are feasible in realtime.
 */
public class RDXFootstepChecker
{
   private final RDX3DPanel primary3DPanel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepSnapAndWiggler snapper;
   private final FootstepPoseHeuristicChecker stepChecker;
   private BipedalFootstepPlannerNodeRejectionReason reason = null;
   private final ArrayList<BipedalFootstepPlannerNodeRejectionReason> reasons = new ArrayList<>();

   // TODO: Swap stance and swing if candidate step for the very first step of the footsteparraylist is going to be on different side compared to swing's side.
   private RigidBodyTransformReadOnly stanceStepPose;
   private RobotSide stanceSide;
   private RigidBodyTransformReadOnly swingStepPose;
   private RobotSide swingSide;

   private String text = null;
   private RDX3DPanelTooltip tooltip;
   private boolean renderTooltip = false;

   public RDXFootstepChecker(RDXBaseUI baseUI,
                             ROS2SyncedRobotModel syncedRobot,
                             SideDependentList<ConvexPolygon2D> footPolygons,
                             FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      this.footPolygons = footPolygons;
      primary3DPanel = baseUI.getPrimary3DPanel();
      tooltip = new RDX3DPanelTooltip(primary3DPanel);
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      this.footstepPlannerParameters = footstepPlannerParameters;
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler(footPolygons);
      snapper = new FootstepSnapAndWiggler(footPolygons, this.footstepPlannerParameters, environmentHandler);
      stepChecker = new FootstepPoseHeuristicChecker(this.footstepPlannerParameters, snapper, registry);
      setInitialFeet();
   }

   public void setInitialFeet()
   {
      swingStepPose = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToRoot();
      swingSide = RobotSide.RIGHT;
      stanceStepPose = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToRoot();
      stanceSide = RobotSide.LEFT;
   }

   public void swapSides()
   {
      swingSide = swingSide.getOppositeSide();
      stanceSide = stanceSide.getOppositeSide();
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

   // Check validity of 1 step
   public void checkValidSingleStep(RecyclingArrayList<RDXInteractableFootstep> stepList,
                                    FramePose3DReadOnly candidateStepPose,
                                    RobotSide candidateStepSide,
                                    int indexOfFootBeingChecked /* list.size() if not placed yet*/)
   {
      // use current stance, swing
      if (indexOfFootBeingChecked == 0)
      {
         // if futureStep has different footSide than current swing, swap current swing and stance.
         if (candidateStepSide != swingSide)
         {
            swapSides();
            reason = stepChecker.checkValidity(candidateStepSide, candidateStepPose, swingStepPose, stanceStepPose);
         }
         else
         {
            reason = stepChecker.checkValidity(candidateStepSide, candidateStepPose, stanceStepPose, swingStepPose);
         }
      }
      // 0th element will be stance, previous stance will be swing
      else if (indexOfFootBeingChecked == 1)
      {
         RDXInteractableFootstep tempStance = stepList.get(0);
         RigidBodyTransformReadOnly tempStanceTransform = tempStance.getFootPose();
         reason = stepChecker.checkValidity(candidateStepSide, candidateStepPose, tempStanceTransform, stanceStepPose);
      }
      else
      {
         reason = stepChecker.checkValidity(candidateStepSide,
                                            candidateStepPose,
                                            stepList.get(indexOfFootBeingChecked - 1).getFootPose(),
                                            stepList.get(indexOfFootBeingChecked - 2).getFootPose());
      }
      reasons.add(reason);
   }

   public void makeWarnings()
   {
      // checkValidStepList(footstepArrayList);
      if (reason != null)
      {
         text = "Rejected for %s.".formatted(reason.name());
      }
      else
      {
         text = "Passes checks.";
      }
   }

   // Should call this in walkFromSteps before clearing the stepList.
   public void clear(RecyclingArrayList<RDXInteractableFootstep> stepList)
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

   public BipedalFootstepPlannerNodeRejectionReason getReason()
   {
      return reason;
   }

   public void setRenderTooltip(boolean renderTooltip)
   {
      this.renderTooltip = renderTooltip;
   }

   public RDX3DPanel getPrimary3DPanel()
   {
      return primary3DPanel;
   }

   public RigidBodyTransformReadOnly getStanceStepPose()
   {
      return stanceStepPose;
   }

   public void setPreviousStepPose(RigidBodyTransformReadOnly previousStepTransform)
   {
      this.stanceStepPose = previousStepTransform;
   }

   public RobotSide getStanceSide()
   {
      return stanceSide;
   }

   public void setStanceSide(RobotSide stanceSide)
   {
      this.stanceSide = stanceSide;
   }

   public RigidBodyTransformReadOnly getSwingStepPose()
   {
      return swingStepPose;
   }

   public void setSwingStepPose(RigidBodyTransformReadOnly swingStepTransform)
   {
      this.swingStepPose = swingStepTransform;
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   public void setSwingSide(RobotSide swingSide)
   {
      this.swingSide = swingSide;
   }

   public void update(RecyclingArrayList<RDXInteractableFootstep> footstepArrayList)
   {
      checkValidStepList(footstepArrayList);
   }
}
