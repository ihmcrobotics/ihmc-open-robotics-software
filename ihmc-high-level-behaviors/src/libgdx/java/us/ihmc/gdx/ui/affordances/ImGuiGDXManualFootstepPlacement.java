package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.UUID;

public class ImGuiGDXManualFootstepPlacement implements RenderableProvider
{
   private boolean isFirstStep = false;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final ImFloat goalZOffset = new ImFloat(0.0f);

   //private GDXUIActionMap placeGoalActionMap;
   private boolean placingGoal = false;
   private Point3D lastObjectIntersection;
   private final Pose3D goalPoseForReading = new Pose3D();
   private final Point3D32 tempSpherePosition = new Point3D32();
   private final Vector3D32 tempRotationVector = new Vector3D32();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private ReferenceFrame referenceFrameFootstep;
   private FramePose3D footTextPose;
   boolean footstepCreated = false;

   private float textHeight = 12;

   private final ArrayList<ImGuiGDXManuallyPlacedFootstep> footstepArrayList = new ArrayList<>();
   private int footstepIndex = -1;
   private GDXImGuiBasedUI baseUI;
   private CommunicationHelper communicationHelper;
   private RobotSide currentFootStepSide;
   private ROS2SyncedRobotModel syncedRobot;

   private ImGuiGDXManuallyPlacedFootstepChecker stepChecker;

   private GDXPose3DGizmo gizmo;
   private ImGui3DViewInput latestInput;
   private GDX3DPanel primary3DPanel;

   private Timer timerFlashingFootsteps = new Timer();
   private boolean flashingFootStepsColorHigh;

   public void create(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.syncedRobot = syncedRobotModel;
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
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

   boolean renderTooltip = false;

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

         double z = (lastObjectIntersection != null ? lastObjectIntersection.getZ() : 0.0) + goalZOffset.get();
         if (footstepArrayList.size() > 0)
         {
            if (ImGui.getIO().getKeyCtrl())
            {
               goalZOffset.set(goalZOffset.get() - (input.getMouseWheelDelta() / 30.0f));
            }

            //Set position of modelInstance, selectablePose3DGizmo, and the sphere used in stepCheckIsPointInsideAlgorithm all to the pointInWorld that the cursor is at
            GDXTools.toGDX(pickPointInWorld, footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform);
            footstepArrayList.get(footstepIndex).setGizmoPose(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ());
            footstepArrayList.get(footstepIndex).getBoundingSphere().getPosition().set(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ());

            // when left button clicked and released.
            if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placingGoal = true;
               footstepCreated = false;

               //Switch sides
               currentFootStepSide = currentFootStepSide.getOppositeSide();
               createNewFootStep(currentFootStepSide);
            }

            // hovering.
            // TODO: (need yaw here?)
            stepChecker.checkValidStep(footstepArrayList, new DiscreteFootstep(pickPointInWorld.getX(), pickPointInWorld.getY(), 0, currentFootStepSide) , placingGoal);

            stepChecker.getInput(input, placingGoal);

			//If out of bounds, flash colors
            footstepArrayList.get(footstepIndex).flashFootstepsWhenBadPlacement(stepChecker, timerFlashingFootsteps, flashingFootStepsColorHigh);
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
         stepChecker.getInput(input, placingGoal);
         stepChecker.checkValidStep(footstepArrayList, new DiscreteFootstep(pickPointInWorld.getX(), pickPointInWorld.getY(), 0, currentFootStepSide) , placingGoal);
         for(int i =0; i<footstepArrayList.size(); i++)
         {
            if (!placingGoal && footstepArrayList.get(i).isPickSelected() && footstepArrayList.size() > 0)
            {
               footstepArrayList.get(i).flashFootstepsWhenBadPlacement(stepChecker, timerFlashingFootsteps, flashingFootStepsColorHigh);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Place footstep:");
      ImGui.sameLine();
      if(ImGui.button(labels.get("Left")))
      {
         createNewFootStep(RobotSide.LEFT);
      }
      ImGui.sameLine();
      if(ImGui.button(labels.get("Right")))
      {
         createNewFootStep(RobotSide.RIGHT);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Walk")))
      {
         if(getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
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

         ImGui.getWindowDrawList().addRectFilled(drawStartX , drawStartY, drawStartX + 150.0f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
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

         controller.getTransformZUpToWorld(footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform);

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
      for (int i =0; i<footstepArrayList.size(); i++)
      {
         footstepArrayList.get(i).update();
      }
   }

   public void clear()
   {
      placingGoal = false;

      while(footstepArrayList.size() >0)
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
         goalZOffset.set((float) pose.getZ());
      }
      goalPoseForReading.set(pose);
   }

   public ArrayList<ImGuiGDXManuallyPlacedFootstep> getFootstepArrayList()
   {
      return footstepArrayList;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      placingGoal = true;
      footstepIndex++;
      footstepArrayList.add(new ImGuiGDXManuallyPlacedFootstep(baseUI, footstepSide, footstepIndex));
      footstepCreated = true;
      currentFootStepSide = footstepSide;
   }

   public void removeFootStep()
   {
      if (footstepArrayList.size() > 0 && footstepArrayList.get(footstepIndex).getFootstepModelInstance() != null)
         footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
      goalZOffset.set(0.0f);

      baseUI.getPrimaryScene().removeRenderableAdapter((footstepArrayList.remove(footstepIndex).getRenderableAdapter()));
      footstepIndex--;

   }


}
