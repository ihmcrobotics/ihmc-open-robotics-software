package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
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
 */
public class GDXFootstepChecker
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

   // TODO: Swap stance and swing if candidate step for the very first step of the footsteparraylist is going to be on different side compared to swing's side.
   private RigidBodyTransform stanceStepTransform;
   private RobotSide stanceSide;
   private RigidBodyTransform swingStepTransform;
   private RobotSide swingSide;

   private String text = null;
   private ImGui3DViewInput latestInput;
   private boolean renderTooltip = false;

   public GDXFootstepChecker(GDXImGuiBasedUI baseUI, ROS2SyncedRobotModel syncedRobot, FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);
      this.footstepPlannerParameters = footstepPlannerParameters;
      snapper = new FootstepSnapAndWiggler(footPolygons, this.footstepPlannerParameters);
      stepChecker = new FootstepPoseHeuristicChecker(this.footstepPlannerParameters, snapper, registry);
      setInitialFeet();
   }

   public void setInitialFeet()
   {
      swingStepTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();
      swingSide = RobotSide.RIGHT;
      stanceStepTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();
      stanceSide = RobotSide.LEFT;
   }

   public void swapSides()
   {
      swingSide = swingSide.getOppositeSide();
      stanceSide = stanceSide.getOppositeSide();
   }

   public void getInput(ImGui3DViewInput input)
   {
      latestInput = input;
   }

   private void renderTooltips()
   {
      if (latestInput != null && renderTooltip)
      {
         float offsetX = 10.0f;
         float offsetY = 31.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + text.length() * 7.2f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());

         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(), ImGuiTools.getSmallFont().getFontSize(), drawStartX + 5.0f, drawStartY + 2.0f, Color.WHITE.toIntBits(), text);
      }
   }

   // TODO: This should update candidate, stance, and swing in the ImGuiGDXManualFootstepPlacement,
   //  updates RejectionReason, and generate warning message in the UI screen.
   public void checkValidStepList(RecyclingArrayList<GDXInteractableFootstep> stepList)
   {
      reasons.clear();
      setInitialFeet();
      // iterate through the list ( + current initial stance and swing) and check validity for all.
      for (int i = 0; i < stepList.size(); ++i)
      {
         checkValidSingleStep(stepList, stepList.get(i).getFootTransformInWorld(), stepList.get(i).getFootstepSide(), i);
      }
   }

   // Check validity of 1 step
   public void checkValidSingleStep(RecyclingArrayList<GDXInteractableFootstep> stepList,
                                    RigidBodyTransform candidateStepTransform,
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
            reason = stepChecker.checkValidity(candidateStepSide, candidateStepTransform, swingStepTransform, stanceStepTransform);
         }
         else
         {
            reason = stepChecker.checkValidity(candidateStepSide, candidateStepTransform, stanceStepTransform, swingStepTransform);
         }
      }
      // 0th element will be stance, previous stance will be swing
      else if (indexOfFootBeingChecked == 1)
      {
         GDXInteractableFootstep tempStance = stepList.get(0);
         RigidBodyTransform tempStanceTransform = tempStance.getFootTransformInWorld();
         reason = stepChecker.checkValidity(candidateStepSide, candidateStepTransform, tempStanceTransform, stanceStepTransform);
      }
      else
      {
         reason = stepChecker.checkValidity(candidateStepSide,
                                            candidateStepTransform,
                                            stepList.get(indexOfFootBeingChecked - 1).getFootTransformInWorld(),
                                            stepList.get(indexOfFootBeingChecked - 2).getFootTransformInWorld());
      }
      reasons.add(reason);
   }

   public void makeWarnings()
   {
      // checkValidStepList(footstepArrayList);
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
   public void clear(RecyclingArrayList<GDXInteractableFootstep> stepList)
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

   public GDX3DPanel getPrimary3DPanel()
   {
      return primary3DPanel;
   }

   public RigidBodyTransform getStanceStepTransform()
   {
      return stanceStepTransform;
   }

   public void setStanceStepTransform(RigidBodyTransform stanceStepTransform)
   {
      this.stanceStepTransform = stanceStepTransform;
   }

   public RobotSide getStanceSide()
   {
      return stanceSide;
   }

   public void setStanceSide(RobotSide stanceSide)
   {
      this.stanceSide = stanceSide;
   }

   public RigidBodyTransform getSwingStepTransform()
   {
      return swingStepTransform;
   }

   public void setSwingStepTransform(RigidBodyTransform swingStepTransform)
   {
      this.swingStepTransform = swingStepTransform;
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   public void setSwingSide(RobotSide swingSide)
   {
      this.swingSide = swingSide;
   }

   public void update(RecyclingArrayList<GDXInteractableFootstep> footstepArrayList)
   {
      checkValidStepList(footstepArrayList);
   }
}
