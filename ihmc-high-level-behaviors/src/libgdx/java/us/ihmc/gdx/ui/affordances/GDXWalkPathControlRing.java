package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPathControlRingGizmo;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicInteger;

public class GDXWalkPathControlRing
{
   private final GDXPathControlRingGizmo footstepPlannerGoalGizmo = new GDXPathControlRingGizmo();
   private boolean selected = false;
   private boolean modified = false;
   private boolean mouseRingPickSelected;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2Helper;
   private MovingReferenceFrame midFeetZUpFrame;
   private ResettableExceptionHandlingExecutorService footstepPlanningThread;
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private GDXFootstepGraphic leftStanceFootstepGraphic;
   private GDXFootstepGraphic rightStanceFootstepGraphic;
   private GDXFootstepGraphic leftGoalFootstepGraphic;
   private GDXFootstepGraphic rightGoalFootstepGraphic;
   private final FramePose3D leftStanceFootPose = new FramePose3D();
   private final FramePose3D rightStanceFootPose = new FramePose3D();
   private final FramePose3D leftGoalFootPose = new FramePose3D();
   private final FramePose3D rightGoalFootPose = new FramePose3D();
   private ReferenceFrame goalFrame;
   private final FramePose3D goalPose = new FramePose3D();
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final FramePose3D startPose = new FramePose3D();
   private SideDependentList<MovingReferenceFrame> footFrames;
   private final AtomicInteger footstepPlannerId = new AtomicInteger(0);
   private FootstepPlanningModule footstepPlanner;
   private GDXFootstepPlanGraphic foostepPlanGraphic;
   private double halfIdealFootstepWidth;
   private volatile FootstepPlan footstepPlan;
   private volatile FootstepPlan footstepPlanToGenerateMeshes;
   private final AxisAngle walkFacingDirection = new AxisAngle();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private boolean useAStarPlanner = false;
   private TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final FootstepPlannerGoal turnWalkTurnGoal = new FootstepPlannerGoal();

   public void create(GDXFocusBasedCamera camera3D, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper ros2Helper)
   {
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
      footstepPlannerGoalGizmo.create(camera3D);
      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      footFrames = syncedRobot.getReferenceFrames().getSoleFrames();

      // periodic thread to footstep plan
      footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("WalkPathControlPlanning", true, 1);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftStanceFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightStanceFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new GDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      goalFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("goalPose",
                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                  footstepPlannerGoalGizmo.getTransformToParent());

      turnWalkTurnPlanner = new TurnWalkTurnPlanner(footstepPlannerParameters);

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      foostepPlanGraphic = new GDXFootstepPlanGraphic(contactPoints);
      leftStanceFootstepGraphic.create();
      rightStanceFootstepGraphic.create();
      leftGoalFootstepGraphic.create();
      rightGoalFootstepGraphic.create();

      halfIdealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth() / 2.0;
      leftStanceFootPose.getPosition().addY(halfIdealFootstepWidth);
      rightStanceFootPose.getPosition().subY(halfIdealFootstepWidth);
      leftStanceFootstepGraphic.setPose(leftStanceFootPose);
      rightStanceFootstepGraphic.setPose(rightStanceFootPose);
   }

   public void update()
   {
      if (!modified)
      {
         footstepPlannerGoalGizmo.getTransformToParent().set(midFeetZUpFrame.getTransformToWorldFrame());
      }

      if (footstepPlanToGenerateMeshes != null)
      {
         foostepPlanGraphic.generateMeshes(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlanToGenerateMeshes,
                                                                                           "Walk Path Control Ring Plan"));
         footstepPlanToGenerateMeshes = null;
      }
      foostepPlanGraphic.update();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean leftMouseReleasedWithoutDrag = input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      footstepPlannerGoalGizmo.process3DViewInput(input);
      mouseRingPickSelected = footstepPlannerGoalGizmo.getHollowCylinderPickSelected();

      if (!modified && mouseRingPickSelected && leftMouseReleasedWithoutDrag)
      {
         selected = true;
         modified = true;
         walkFacingDirection.set(Axis3D.Z, 0.0);
         updateStuff();
         queueFootstepPlan();
      }
      if (selected && !footstepPlannerGoalGizmo.getAnyPartPickSelected() && leftMouseReleasedWithoutDrag)
      {
         selected = false;
      }
      if (modified && mouseRingPickSelected && leftMouseReleasedWithoutDrag)
      {
         selected = true;
      }

      if (modified)
      {
         updateStuff();
      }
      if (selected && leftMouseReleasedWithoutDrag)
      {
         if (footstepPlannerGoalGizmo.getPositiveXArrowPickSelected())
         {
            walkFacingDirection.set(Axis3D.Z, 0.0);
         }
         else if (footstepPlannerGoalGizmo.getPositiveYArrowPickSelected())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI / 2.0);
         }
         else if (footstepPlannerGoalGizmo.getNegativeXArrowPickSelected())
         {
            walkFacingDirection.set(Axis3D.Z, Math.PI);
         }
         else if (footstepPlannerGoalGizmo.getNegativeYArrowPickSelected())
         {
            walkFacingDirection.set(Axis3D.Z, -Math.PI / 2.0);
         }
         if (footstepPlannerGoalGizmo.getAnyArrowPickSelected())
         {
            footstepPlannerGoalGizmo.getTransformToParent().appendOrientation(walkFacingDirection);
            updateStuff();
            queueFootstepPlan();
         }
      }
      if (selected && footstepPlannerGoalGizmo.isBeingDragged())
      {
         queueFootstepPlan();
      }
      if (selected && ImGui.isKeyReleased(ImGuiTools.getSpaceKey()))
      {
         // Send footsteps to robot
         double swingDuration = 1.2;
         double transferDuration = 0.8;
         FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                                                       swingDuration,
                                                                                                                       transferDuration);
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
         ros2Helper.publishToController(footstepDataListMessage);
      }

      if (modified && selected && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         selected = false;
         modified = false;
         foostepPlanGraphic.clear();
      }
      if (selected && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         selected = false;
      }

      footstepPlannerGoalGizmo.setShowArrows(selected);
      footstepPlannerGoalGizmo.setHighlightingEnabled(modified);
   }

   private void queueFootstepPlan()
   {
      footstepPlanningThread.clearQueueAndExecute(() ->
      {
         if (useAStarPlanner)
         {
            planFoostepsUsingAStarPlanner(new Pose3D(leftStanceFootPose),
                                          new Pose3D(rightStanceFootPose),
                                          new Pose3D(leftGoalFootPose),
                                          new Pose3D(rightGoalFootPose));
         }
         else
         {
            planFootstepsUsingTurnWalkTurnPlanner();
         }
      });
   }

   private void updateStuff()
   {
      goalFrame.update();
      goalPose.setToZero(goalFrame);
//      goalPose.appendRotation(walkFacingDirection);

      leftGoalFootPose.setIncludingFrame(goalPose);
      leftGoalFootPose.getPosition().addY(halfIdealFootstepWidth);
      leftGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightGoalFootPose.setIncludingFrame(goalPose);
      rightGoalFootPose.getPosition().subY(halfIdealFootstepWidth);
      rightGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      leftStanceFootPose.setToZero(footFrames.get(RobotSide.LEFT));
      leftStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightStanceFootPose.setToZero(footFrames.get(RobotSide.RIGHT));
      rightStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      double lowestStanceZ = Math.min(leftStanceFootPose.getZ(), rightStanceFootPose.getZ());
      leftStanceFootPose.setZ(lowestStanceZ);
      rightStanceFootPose.setZ(lowestStanceZ);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      midFeetZUpPose.setToZero(midFeetZUpFrame);
      midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
      startPose.setToZero(midFeetZUpFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      startPose.getOrientation().set(goalPose.getOrientation());

      leftGoalFootstepGraphic.setPose(leftGoalFootPose);
      rightGoalFootstepGraphic.setPose(rightGoalFootPose);
   }

   private void planFootstepsUsingTurnWalkTurnPlanner()
   {
      turnWalkTurnGoal.setGoalPoseBetweenFeet(goalPose);
      turnWalkTurnPlanner.setGoal(turnWalkTurnGoal);
      turnWalkTurnPlanner.setInitialStanceFoot(leftStanceFootPose, RobotSide.LEFT); // TODO: Furthest away
      turnWalkTurnPlanner.plan();
      footstepPlan = footstepPlanToGenerateMeshes = turnWalkTurnPlanner.getPlan();
   }

   private void planFoostepsUsingAStarPlanner(Pose3DReadOnly leftStanceFootPose,
                                              Pose3DReadOnly rightStanceFootPose,
                                              Pose3DReadOnly leftGoalFootPose,
                                              Pose3DReadOnly rightGoalFootPose)
   {
      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.getBodyPathWaypoints().add(midFeetZUpPose);
      footstepPlannerRequest.getBodyPathWaypoints().add(startPose);
      footstepPlannerRequest.getBodyPathWaypoints().add(goalPose);
      footstepPlannerRequest.setStartFootPoses(leftStanceFootPose, rightStanceFootPose);
      footstepPlannerRequest.setGoalFootPoses(leftGoalFootPose, rightGoalFootPose);
      footstepPlannerRequest.setAssumeFlatGround(true);
      footstepPlannerRequest.setRequestId(footstepPlannerId.getAndIncrement());
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      footstepPlan = footstepPlanToGenerateMeshes = new FootstepPlan(footstepPlannerOutput.getFootstepPlan());
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get("Turn Walk Turn"), !useAStarPlanner))
      {
         useAStarPlanner = false;
      }
      if (ImGui.radioButton(labels.get("A* Planner"), useAStarPlanner))
      {
         useAStarPlanner = true;
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modified)
      {
         leftStanceFootstepGraphic.getRenderables(renderables, pool);
         rightStanceFootstepGraphic.getRenderables(renderables, pool);
         leftGoalFootstepGraphic.getRenderables(renderables, pool);
         rightGoalFootstepGraphic.getRenderables(renderables, pool);
         foostepPlanGraphic.getRenderables(renderables, pool);
      }
      if (modified || mouseRingPickSelected)
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
      }
   }

   public void clearGraphics()
   {
      leftStanceFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      rightStanceFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      leftGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      rightGoalFootstepGraphic.setPose(BehaviorTools.createNaNPose());
      foostepPlanGraphic.clear();
   }

   public void destroy()
   {
      footstepPlanningThread.destroy();
      foostepPlanGraphic.destroy();
   }
}
