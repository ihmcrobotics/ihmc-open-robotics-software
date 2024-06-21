package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

import java.util.ArrayList;

public class RDXWalkPathControlRing
{
   private final RDXSelectablePathControlRingGizmo footstepPlannerGoalGizmo = new RDXSelectablePathControlRingGizmo();
   private final Notification becomesModifiedNotification = new Notification();
   private final Notification goalUpdatedNotification = new Notification();
   private MovingReferenceFrame midFeetZUpFrame;
   private RDXFootstepGraphic leftGoalFootstepGraphic;
   private RDXFootstepGraphic rightGoalFootstepGraphic;
   private final FramePose3D leftGoalFootPose = new FramePose3D();
   private final FramePose3D rightGoalFootPose = new FramePose3D();
   private double halfIdealFootstepWidth;
   private final AxisAngle walkFacingDirection = new AxisAngle();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDX3DPanelTooltip tooltip;

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      DefaultFootstepPlannerParametersBasics footstepPlannerParameters)
   {
      footstepPlannerGoalGizmo.create(panel3D);
      panel3D.addImGuiOverlayAddition(this::renderTooltips);
      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      tooltip = new RDX3DPanelTooltip(panel3D);

      leftGoalFootstepGraphic.create();
      rightGoalFootstepGraphic.create();

      halfIdealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth() / 2.0;

      RDXBaseUI.getInstance().getKeyBindings().register("Deleted selected control ring", "Delete");
      RDXBaseUI.getInstance().getKeyBindings().register("Unselect control ring", "Escape");
   }

   public void update()
   {
      if (!footstepPlannerGoalGizmo.getModified())
      {
         footstepPlannerGoalGizmo.getPathControlRingGizmo().getTransformToParent().set(midFeetZUpFrame.getTransformToWorldFrame());
      }
      else
      {
         updateStuff();
      }
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.calculateVRPick(vrContext);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.processVRInput(vrContext);
      boolean selected = footstepPlannerGoalGizmo.getSelected();

      if (selected)
      {
         becomeModified(true);
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      tooltip.setInput(input);
      boolean leftMouseReleasedWithoutDrag = input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      boolean previouslySelected = footstepPlannerGoalGizmo.getSelected();
      footstepPlannerGoalGizmo.process3DViewInput(input);
      boolean newlySelected = footstepPlannerGoalGizmo.getSelected() && !previouslySelected;

      if (newlySelected)
      {
         becomeModified(true);
      }

      if (footstepPlannerGoalGizmo.getSelected() && leftMouseReleasedWithoutDrag)
      {
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getPositiveXArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, 0.0);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getPositiveYArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI / 2.0);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getNegativeXArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI);
         }
         else if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getNegativeYArrowHovered())
         {
            walkFacingDirection.set(Axis3D.Z, -Math.PI / 2.0);
         }
         if (footstepPlannerGoalGizmo.getPathControlRingGizmo().getAnyArrowHovered())
         {
            footstepPlannerGoalGizmo.getPathControlRingGizmo().getTransformToParent().appendOrientation(walkFacingDirection);
            footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoModifiedByUser().set();
            footstepPlannerGoalGizmo.getPathControlRingGizmo().update();
            footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoModifiedByUser().set();
         }
      }
      if (footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoModifiedByUser().poll())
      {
         goalUpdatedNotification.set();
      }

      if (footstepPlannerGoalGizmo.getModified() && footstepPlannerGoalGizmo.getSelected() && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         delete();
      }
      if (footstepPlannerGoalGizmo.getSelected() && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         footstepPlannerGoalGizmo.setSelected(false);
      }
   }

   private void updateStuff()
   {
      leftGoalFootPose.setToZero(footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoFrame());
      leftGoalFootPose.getPosition().addY(halfIdealFootstepWidth);
      leftGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightGoalFootPose.setToZero(footstepPlannerGoalGizmo.getPathControlRingGizmo().getGizmoFrame());
      rightGoalFootPose.getPosition().subY(halfIdealFootstepWidth);
      rightGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      leftGoalFootstepGraphic.setPose(leftGoalFootPose);
      rightGoalFootstepGraphic.setPose(rightGoalFootPose);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Control ring:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Deleted"), !footstepPlannerGoalGizmo.getSelected() && !footstepPlannerGoalGizmo.getModified()))
      {
         delete();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Modified"), !footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getModified()))
      {
         becomeModified(false);
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getModified()))
      {
         becomeModified(true);
      }
   }

   public void becomeModified(boolean selected)
   {
      footstepPlannerGoalGizmo.setSelected(selected);
      if (!footstepPlannerGoalGizmo.getModified())
      {
         footstepPlannerGoalGizmo.setModified(true);
         becomesModifiedNotification.set();
         walkFacingDirection.set(Axis3D.Z, 0.0);
         goalUpdatedNotification.set();
      }
   }

   private void renderTooltips()
   {
      if (footstepPlannerGoalGizmo.getSelected() && footstepPlannerGoalGizmo.getPathControlRingGizmo().getAnyPartHovered())
      {
         tooltip.render("""
                          Use left mouse drag to translate.
                          Use right mouse drag to yaw.
                          Use keyboard arrows to translate. (Hold shift for slow)
                          Use alt+left and alt+left arrows to yaw. (Hold shift for slow)
                          """);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (footstepPlannerGoalGizmo.getModified())
      {
         leftGoalFootstepGraphic.getRenderables(renderables, pool);
         rightGoalFootstepGraphic.getRenderables(renderables, pool);
      }
      footstepPlannerGoalGizmo.getVirtualRenderables(renderables, pool);
   }

   public void delete()
   {
      footstepPlannerGoalGizmo.setSelected(false);
      footstepPlannerGoalGizmo.setModified(false);
      clearGraphics();
   }

   public void clearGraphics()
   {
      leftGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      rightGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
   }

   public boolean isSelected()
   {
      return footstepPlannerGoalGizmo.getSelected();
   }

   public Notification getBecomesModifiedNotification()
   {
      return becomesModifiedNotification;
   }

   public Notification getGoalUpdatedNotification()
   {
      return goalUpdatedNotification;
   }

   public Pose3DReadOnly getGoalPose()
   {
      return footstepPlannerGoalGizmo.getPathControlRingGizmo().getPose3D();
   }

   public RDXSelectablePathControlRingGizmo getFootstepPlannerGoalGizmo()
   {
      return footstepPlannerGoalGizmo;
   }
}