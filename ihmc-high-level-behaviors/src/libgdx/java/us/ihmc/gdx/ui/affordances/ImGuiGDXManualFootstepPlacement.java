package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class ImGuiGDXManualFootstepPlacement implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private boolean placingGoal = false;
   private final Pose3D goalPoseForReading = new Pose3D();
   private final ArrayList<ImGuiGDXManuallyPlacedFootstep> footstepArrayList = new ArrayList<>();
   private int footstepIndex = -1;
   private GDXImGuiBasedUI baseUI;
   private CommunicationHelper communicationHelper;
   private RobotSide currentFootStepSide;
   private ROS2SyncedRobotModel syncedRobot;
   private ImGuiGDXManuallyPlacedFootstepChecker stepChecker;
   private ImGui3DViewInput latestInput;
   private GDX3DPanel primary3DPanel;
   private boolean renderTooltip = false;
   FramePose3D tempFramePose = new FramePose3D();



   public void create(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addWindowDrawListAddition(this::renderTooltips);

      stepChecker = new ImGuiGDXManuallyPlacedFootstepChecker(baseUI, communicationHelper, syncedRobot);
      clear();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      renderTooltip = false;

      for (ImGuiGDXManuallyPlacedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.calculate3DViewPick(input);
      }
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;

      for (ImGuiGDXManuallyPlacedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.process3DViewInput(input);
      }

      if (placingGoal && input.isWindowHovered())
      {
         Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
         renderTooltip = true;

         if (footstepArrayList.size() > 0)
         {
            //Set position of modelInstance, selectablePose3DGizmo, and the sphere used in stepCheckIsPointInsideAlgorithm all to the pointInWorld that the cursor is at
            GDXTools.toGDX(pickPointInWorld, footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform);

            footstepArrayList.get(footstepIndex).setGizmoPose(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ(),
                                                              footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());

            footstepArrayList.get(footstepIndex)
                             .getBoundingSphere()
                             .getPosition()
                             .set(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ());

            // when left button clicked and released.
            if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placingGoal = true;

               //Switch sides
               currentFootStepSide = currentFootStepSide.getOppositeSide();
               createNewFootStep(currentFootStepSide);
            }



            // hovering.
            // TODO: (need yaw here?)
            stepChecker.checkValidStep(footstepArrayList,
                                       new DiscreteFootstep(pickPointInWorld.getX(), pickPointInWorld.getY(), 0, currentFootStepSide),
                                       placingGoal);

            stepChecker.getInput(input);
         }

         if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            placingGoal = false;

            removeFootStep();
         }
      }
      if (input.isWindowHovered())
      {
         Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
         renderTooltip = true;

         // hovering.
         // TODO: (need yaw here?)
         stepChecker.getInput(input);
         stepChecker.checkValidStep(footstepArrayList,
                                    new DiscreteFootstep(pickPointInWorld.getX(), pickPointInWorld.getY(), 0, currentFootStepSide),
                                    placingGoal);
         ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
         for (int i = 0; i < temporaryReasons.size(); i++)
         {
            footstepArrayList.get(i).flashFootstepsWhenBadPlacement(temporaryReasons.get(i));
         }
      }

      if (footstepArrayList.size()>0)
      {
         for (int i = 0; i < footstepArrayList.size(); ++i)
         {
            if (footstepArrayList.get(i).isPickSelected())
            {
               stepChecker.setReasonFrom(i);
               break;
            }
         }
      }

      stepChecker.makeWarnings();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Place footstep:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Left")))
      {
         createNewFootStep(RobotSide.LEFT);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Right")))
      {
         createNewFootStep(RobotSide.RIGHT);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Walk")))
      {
         if (getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
      }

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
   }

   private void renderTooltips()
   {
      if (renderTooltip)
      {
         float offsetX = 10.0f;
         float offsetY = 10.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 150.0f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(),
                       ImGuiTools.getSmallFont().getFontSize(),
                       drawStartX + 5.0f,
                       drawStartY + 2.0f,
                       Color.WHITE.toIntBits(),
                       "Right click to exit");
      }
   }

   public void handleVREvents(GDXVRManager vrManager)
   {
      vrManager.getContext().getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
         if (triggerClick.bChanged() && triggerClick.bState())
         {
            placingGoal = true;
         }
         if (triggerClick.bChanged() && !triggerClick.bState())
         {
            placingGoal = false;
         }

         controller.getTransformZUpToWorld(footstepArrayList.get(footstepIndex)
                                                            .getFootstepModelInstance().transform);
      });
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

      if (isPlaced())
      {
         for (int i = 0; i < footstepArrayList.size(); i++)
         {
            footstepArrayList.get(i).getVirtualRenderables(renderables, pool);
            footstepArrayList.get(i).getFootstepModelInstance().getRenderables(renderables, pool);
         }
      }
   }

   public boolean isPlaced()
   {
      if (footstepArrayList.size() <= 0)
      {
         return false;
      }
      else
      {
         return !Float.isNaN(footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform.val[Matrix4.M03]);
      }
   }

   public void update()
   {
      for (int i = 0; i < footstepArrayList.size(); i++)
      {
         footstepArrayList.get(i).update();
      }
   }

   public void clear()
   {
      placingGoal = false;

      while (footstepArrayList.size() > 0)
      {
         removeFootStep();
      }

      footstepArrayList.clear();
      footstepIndex = -1;
   }

   private void walkFromSteps()
   {
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      for (ImGuiGDXManuallyPlacedFootstep step : footstepArrayList)
      {
         generateFootStepDataMessage(messageList, step);
         messageList.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
      communicationHelper.publishToController(messageList);
      // done walking >>
      // set stance and swing as last two steps of the footstepArrayList (if this list is not empty)
      // delete steps in singleFootStepAffordance.
      stepChecker.clear(footstepArrayList);
      clear();
   }

   private void generateFootStepDataMessage(FootstepDataListMessage messageList, ImGuiGDXManuallyPlacedFootstep step)
   {
      FootstepDataMessage stepMessage = messageList.getFootstepDataList().add();
      stepMessage.setRobotSide(step.getFootstepSide().toByte());
      stepMessage.getLocation().set(new Point3D(step.getSelectablePose3DGizmo().getPoseGizmo().getPose().getPosition()));
      stepMessage.getOrientation().set(step.getPose().getOrientation());
      stepMessage.setSwingDuration(1.2);
      stepMessage.setTransferDuration(0.8);
   }

   public Pose3DReadOnly getGoalPose()
   {
      return goalPoseForReading;
   }

   public void setGoalPoseAndPassOn(Pose3DReadOnly pose)
   {
      setGoalPoseNoCallbacks(pose);
   }

   public void setGoalPoseNoCallbacks(Pose3DReadOnly pose)
   {
      if (pose == null)
      {
         clear();
      }
      else
      {
         GDXTools.toGDX(pose.getPosition(), footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform);
      }
      goalPoseForReading.set(pose);
   }

   public ArrayList<ImGuiGDXManuallyPlacedFootstep> getFootstepArrayList()
   {
      return footstepArrayList;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      RigidBodyTransform latestFootstepTransform = getLatestFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();
      placingGoal = true;
      footstepIndex++;
      footstepArrayList.add(new ImGuiGDXManuallyPlacedFootstep(baseUI, footstepSide, footstepIndex));
      currentFootStepSide = footstepSide;

      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().set(new RotationMatrix(latestFootstepYaw, 0.0, 0.0));
      tempFramePose.get(footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
      footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();


   }

   public void removeFootStep()
   {
      if (footstepArrayList.size() > 0 && footstepArrayList.get(footstepIndex).getFootstepModelInstance() != null)
         footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;

      baseUI.getPrimaryScene().removeRenderableAdapter((footstepArrayList.remove(footstepIndex).getRenderableAdapter()));
      footstepIndex--;
   }

   public RigidBodyTransform getLatestFootstepTransform(RobotSide robotSide)
   {
      if(footstepArrayList.size()>0)
      {
         return footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent();
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }

   }
}
