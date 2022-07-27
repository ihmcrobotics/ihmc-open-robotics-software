package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

/**
 * Helps the operator confirm that manually placed footsteps are feasible in realtime.
 *
 * FIXME: This doesn't include checks like "step is too high" or "step is too low".
 * TODO: Figure out how the footstep planner does that and use that.
 */
public class ImGuiGDXManuallyPlacedFootstepChecker
{
   private final GDX3DPanel primary3DPanel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private final FootstepSnapAndWiggler snapper;
   private final FootstepPoseHeuristicChecker stepChecker;
   private BipedalFootstepPlannerNodeRejectionReason reason = null;
   private final ArrayList<BipedalFootstepPlannerNodeRejectionReason> reasons = new ArrayList<>();

   // TODO: swap stance and swing if candidate step for the very first step of the footsteparraylist is going to be on different side compared to swing's side.
   private DiscreteFootstep stance = new DiscreteFootstep(0, 0, 0, RobotSide.RIGHT);
   private DiscreteFootstep swing = new DiscreteFootstep(0, 0, 0, RobotSide.LEFT);
   private String text = null;
   private ImGui3DViewInput latestInput;
   boolean renderTooltip = false;
   private ArrayList<ImGuiGDXManuallyPlacedFootstep> plannedSteps;

   public ImGuiGDXManuallyPlacedFootstepChecker(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addWindowDrawListAddition(this::renderTooltips);
      footstepPlannerParameters = communicationHelper.getRobotModel().getFootstepPlannerParameters();
      snapper = new FootstepSnapAndWiggler(footPolygons, footstepPlannerParameters);
      stepChecker = new FootstepPoseHeuristicChecker(footstepPlannerParameters, snapper, registry);
      setInitialFeet();
   }

    public void setInitialFeet()
    {
        RigidBodyTransform initialRightFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();
        RigidBodyTransform initialLeftFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();

        Vector3DBasics initialRightFoot = initialRightFootTransform.getTranslation();
        Vector3DBasics initialLeftFoot = initialLeftFootTransform.getTranslation();

        swing = new DiscreteFootstep(initialLeftFoot.getX(), initialLeftFoot.getY(), initialLeftFootTransform.getRotation().getYaw(), RobotSide.LEFT);
        stance = new DiscreteFootstep(initialRightFoot.getX(), initialRightFoot.getY(), initialRightFootTransform.getRotation().getYaw(), RobotSide.RIGHT);
    }

   public void getInput(ImGui3DViewInput input)
   {
      latestInput = input;
   }

   private void renderTooltips()
   {
      if (this.latestInput != null && plannedSteps.size() > 0)
      {
         float offsetX = 10.0f;
         float offsetY = 31.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 120.0f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());

         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(), ImGuiTools.getSmallFont().getFontSize(), drawStartX + 5.0f, drawStartY + 2.0f, Color.WHITE.toIntBits(), text);
      }
   }

   // TODO: This should update candidate, stance, and swing in the ImGuiGDXManualFootstepPlacement,
   //  updates RejectionReason, and generate warning message in the UI screen.
   public void checkValidStep(ArrayList<ImGuiGDXManuallyPlacedFootstep> stepList, DiscreteFootstep futureStep, boolean placingGoal)
   {
      plannedSteps = stepList;
      reasons.clear();
      setInitialFeet();
      // iterate through the list ( + current initial stance and swing) and check validity for all.
      for (int i = 0; i < stepList.size(); ++i)
      {
         DiscreteFootstep candidate = convertToDiscrete(stepList.get(i));
         // use current stance, swing
         if (i == 0)
         {
            // if futureStep has different footSide than current swing, swap current swing and stance.
            if (candidate.getRobotSide() != swing.getRobotSide())
            {
               swapSteps();
            }
            reason = stepChecker.checkStepValidity(candidate, stance, swing);
         }
         // 0th element will be stance, previous stance will be swing
         else if (i == 1)
         {
            DiscreteFootstep temp = convertToDiscrete(stepList.get(0));
            reason = stepChecker.checkStepValidity(candidate, temp, stance);
         }
         else
         {
            reason = stepChecker.checkStepValidity(candidate, convertToDiscrete(stepList.get(i - 1)), convertToDiscrete(stepList.get(i - 2)));
         }

         reasons.add(reason);
      }

      if (placingGoal)
      {
         int size = stepList.size();
         DiscreteFootstep tempStance = null;
         DiscreteFootstep tempSwing = null;
         if (stepList.size() >= 2)
         {
            tempStance = convertToDiscrete(stepList.get(size - 1));
            tempSwing = convertToDiscrete(stepList.get(size - 2));
         }
         else if (stepList.size() == 1)
         {
            DiscreteFootstep lastStep = convertToDiscrete(stepList.get(0));
            tempStance = lastStep;
            tempSwing = stance;
         }
         else
         {
            tempStance = stance;
            tempSwing = swing;
         }

         if (tempSwing.getRobotSide() != futureStep.getRobotSide())
         {
            // swap
            DiscreteFootstep temp = tempStance;
            tempStance = tempSwing;
            tempSwing = temp;
         }
         reason = stepChecker.checkStepValidity(futureStep, tempStance, tempSwing);
      }
      renderTooltip = placingGoal;
   }

   // TODO: This should be used when first step of the manual step cycle has different RobotSide than current swing.
   public void swapSteps()
   {
      DiscreteFootstep temp = stance;
      stance = swing;
      swing = temp;
   }

   public DiscreteFootstep convertToDiscrete(ImGuiGDXManuallyPlacedFootstep step)
   {
      Pose3DReadOnly pose = step.getPose();
      Point3DReadOnly position = pose.getPosition();
      return new DiscreteFootstep(position.getX(), position.getY(), step.getPose().getOrientation().getYaw(), step.getFootstepSide());
   }

   public void makeWarnings()
   {
      if (reason != null)
      {
         text = " Warning ! : " + reason.name();
      }
      else
      {
         text = "Looks Good !";
      }
   }

   // Should call this in walkFromSteps before clearing the stepList.
   public void clear(ArrayList<ImGuiGDXManuallyPlacedFootstep> stepList)
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
}
