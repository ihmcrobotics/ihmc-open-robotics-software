package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPathControlRingGizmo;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.PathTypeStepParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

public class GDXWalkPathControlRing implements PathTypeStepParameters
{
   private final GDXPathControlRingGizmo footstepPlannerGoalGizmo = new GDXPathControlRingGizmo();
   private boolean selected = false;
   private boolean modified = false;
   private boolean newlyModified = false;
   private boolean mouseRingPickSelected;
   private GDX3DPanel panel3D;
   private GDXTeleoperationParameters teleoperationParameters;
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
   private int plannerToUse = 0;
   private TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final FootstepPlannerGoal turnWalkTurnGoal = new FootstepPlannerGoal();
   private TurnStraightTurnFootstepGenerator turnStraightTurnFootstepGenerator;
   private SimplePathParameters turnStraightTurnParameters;
   private SteppingParameters steppingParameters;
   private GDXWalkPathType walkPathType = GDXWalkPathType.STRAIGHT;
   private ImGui3DViewInput latestInput;

   private GDXPoseTracking poseTracking;
   private final boolean isContinuousStepping = false;
   private boolean joystickOn = false;
   private ImBoolean joystickMode = new ImBoolean(false);

   public void create(GDXImGuiBasedUI baseUI,
                      GDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      GDXTeleoperationParameters teleoperationParameters,
                      CommunicationHelper communicationHelper,
                      ROS2ControllerHelper ros2Helper,
                      FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.panel3D = panel3D;
      this.teleoperationParameters = teleoperationParameters;
      footstepPlannerGoalGizmo.create(panel3D.getCamera3D());
      panel3D.addImGuiOverlayAddition(this::renderTooltips);
      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      footFrames = syncedRobot.getReferenceFrames().getSoleFrames();

      // periodic thread to footstep plan
      footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("WalkPathControlPlanning", true, 1);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      steppingParameters = robotModel.getWalkingControllerParameters().getSteppingParameters();
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

      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>(syncedRobot.getReferenceFrames().getSoleFrames());
      turnStraightTurnFootstepGenerator = new TurnStraightTurnFootstepGenerator(new SideDependentList<>(null, null),
                                                                                soleFrames,
                                                                                new FramePose2D(),
                                                                                this);
      poseTracking = new GDXPoseTracking(baseUI, robotModel, syncedRobot, ros2Helper, communicationHelper, footstepPlannerParameters);
   }

   public void update(GDXInteractableFootstepPlan plannedFootstepPlacement)
   {
      if (!modified)
      {
         footstepPlannerGoalGizmo.getTransformToParent().set(midFeetZUpFrame.getTransformToWorldFrame());
      }


      if (footstepPlanToGenerateMeshes != null)
//      if (footstepPlan != null)
      {
         plannedFootstepPlacement.updateFromPlan(footstepPlan);
//         foostepPlanGraphic.generateMeshes(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlanToGenerateMeshes,
//                                                                                           "Walk Path Control Ring Plan"));
         footstepPlanToGenerateMeshes = null;
      }
      foostepPlanGraphic.update();

      if (joystickOn)
      {
         poseTracking.setMode(joystickMode.get());
         poseTracking.run(goalPose);
         if (poseTracking.isJoystickMode())
            setGoalGizmoFromJoystickPose();
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;
      boolean leftMouseReleasedWithoutDrag = input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      footstepPlannerGoalGizmo.process3DViewInput(input, selected);
      mouseRingPickSelected = footstepPlannerGoalGizmo.getHollowCylinderPickSelected();

      if (!modified && mouseRingPickSelected && leftMouseReleasedWithoutDrag)
      {
         becomeModified();
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
            walkPathType = GDXWalkPathType.STRAIGHT;
            walkFacingDirection.set(Axis3D.Z, 0.0);
         }
         else if (footstepPlannerGoalGizmo.getPositiveYArrowPickSelected())
         {
            walkPathType = GDXWalkPathType.LEFT_SHUFFLE;
            walkFacingDirection.set(Axis3D.Z, Math.PI / 2.0);
         }
         else if (footstepPlannerGoalGizmo.getNegativeXArrowPickSelected())
         {
            walkPathType = GDXWalkPathType.REVERSE;
            walkFacingDirection.set(Axis3D.Z, Math.PI);
         }
         else if (footstepPlannerGoalGizmo.getNegativeYArrowPickSelected())
         {
            walkPathType = GDXWalkPathType.RIGHT_SHUFFLE;
            walkFacingDirection.set(Axis3D.Z, -Math.PI / 2.0);
         }
         if (footstepPlannerGoalGizmo.getAnyArrowPickSelected())
         {
            footstepPlannerGoalGizmo.getTransformToParent().appendOrientation(walkFacingDirection);
            updateStuff();
            queueFootstepPlan();
         }
      }

      if (selected && joystickOn)
      {
         poseTracking.run(footstepPlannerGoalGizmo.getFramePose3D());
      }

      if (selected && footstepPlannerGoalGizmo.isNewlyModified())
      {
         queueFootstepPlan();
      }

      if (modified && selected && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         delete();
      }
      if (selected && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         selected = false;
      }

      footstepPlannerGoalGizmo.setShowArrows(selected);
      footstepPlannerGoalGizmo.setHighlightingEnabled(modified);
   }

   private void becomeModified()
   {
      selected = true;
      modified = true;
      newlyModified = true;
      walkFacingDirection.set(Axis3D.Z, 0.0);
      updateStuff();
      queueFootstepPlan();
   }

   public boolean checkIsNwlyModified()
   {
      boolean newlyModifiedReturn = newlyModified;
      newlyModified = false;
      return newlyModifiedReturn;
   }

   private void queueFootstepPlan()
   {
      footstepPlanningThread.clearQueueAndExecute(() ->
      {
         if (plannerToUse == 0)
         {
            planFoostepsUsingAStarPlanner(new Pose3D(leftStanceFootPose),
                                          new Pose3D(rightStanceFootPose),
                                          new Pose3D(leftGoalFootPose),
                                          new Pose3D(rightGoalFootPose));
         }
         else if (plannerToUse == 1)
         {
            planFootstepsUsingTurnWalkTurnPlanner();
         }
         else if (plannerToUse == 2)
         {
            planFootstepsUsingTurnStraightTurnFootstepGenerator();
         }
         else
         {
            planJoystick();
            joystickOn = true;
//            footstepPlannerGoalGizmo.setJoystickMode(true);
         }
         if (plannerToUse!=3)
         {
            poseTracking.stop();
            joystickOn = false;
//            footstepPlannerGoalGizmo.setJoystickMode(false);
         }
      });
   }

   public void updateStuff()
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

   public void planFootstepsUsingTurnStraightTurnFootstepGenerator()
   {
      turnStraightTurnFootstepGenerator.setStanceStartPreference(RobotSide.LEFT);
      // TODO:
//      turnStraightTurnFootstepGenerator.setFootstepPath(new TurnStraightTurnOverheadPath());

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

   private void setGoalGizmoFromJoystickPose()
   {
      FramePose3DReadOnly joystickPose = poseTracking.getCurrentPose();
      footstepPlannerGoalGizmo.getTransformToParent().set(new RigidBodyTransform(joystickPose));
      footstepPlannerGoalGizmo.updateTransforms();
   }

   private void planJoystick()
   {
      poseTracking.initiate();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get("A* Planner"), plannerToUse == 0))
      {
         plannerToUse = 0;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Turn Walk Turn"), plannerToUse == 1))
      {
         plannerToUse = 1;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Turn Straight Turn"), plannerToUse == 2))
      {
         plannerToUse = 2;
      }
      if (ImGui.radioButton(labels.get("Continuous Tracking"), plannerToUse == 3 ))
      {
         plannerToUse = 3;
      }
      ImGui.checkbox(labels.get("joystick"), joystickMode);

      ImGuiTools.previousWidgetTooltip("WARNING!! Do not check this box if you don't have xbox controller connected!");
      ImGui.text("Control ring:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Deleted"), !selected && !modified))
      {
         delete();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Modified"), !selected && modified))
      {
         selected = false;
         if (!modified)
         {
            becomeModified();
         }
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), selected && modified))
      {
         selected = true;
         if (!modified)
         {
            becomeModified();
         }
      }
   }

   private void renderTooltips()
   {
      if (selected && footstepPlannerGoalGizmo.getGizmoHovered())
      {
         float offsetX = 10.0f;
         float offsetY = 10.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = panel3D.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = panel3D.getWindowDrawMinY() + mousePosY + offsetY;

         String message = """
                          Use left mouse drag to translate.
                          Use right mouse drag to yaw.
                          Use keyboard arrows to translate. (Hold shift for slow)
                          Use alt+left and alt+left arrows to yaw. (Hold shift for slow)
                          """;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 62 * 6.7f, drawStartY + 4 * 17.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(),
                       ImGuiTools.getSmallFont().getFontSize(),
                       drawStartX + 5.0f,
                       drawStartY + 2.0f,
                       Color.WHITE.toIntBits(),
                       message);
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
      if (joystickOn)
      {
         poseTracking.getRenderables(renderables, pool);
      }
   }

   public void delete()
   {
      selected = false;
      modified = false;
      poseTracking.stop();
      clearGraphics();
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

   @Override
   public double getAngle()
   {
      return switch (walkPathType)
      {
         case STRAIGHT -> 0.0;
         case LEFT_SHUFFLE -> -Math.PI / 2.0;
         case RIGHT_SHUFFLE -> Math.PI / 2.0;
         case REVERSE -> Math.PI;
      };
   }

   @Override
   public double getStepWidth()
   {
      return switch (walkPathType)
      {
         case STRAIGHT -> teleoperationParameters.getStraightStepWidth();
         case LEFT_SHUFFLE, RIGHT_SHUFFLE -> teleoperationParameters.getShuffleStepWidth();
         case REVERSE -> teleoperationParameters.getReverseStepWidth();
      };
   }

   @Override
   public double getStepLength()
   {
      return switch (walkPathType)
      {
         case STRAIGHT -> teleoperationParameters.getFootstepLengthMultiplier() * teleoperationParameters.getStraightStepLength();
         case LEFT_SHUFFLE, RIGHT_SHUFFLE -> teleoperationParameters.getFootstepLengthMultiplier() * teleoperationParameters.getShuffleStepLength();
         case REVERSE -> teleoperationParameters.getFootstepLengthMultiplier() * teleoperationParameters.getReverseStepLength();
      };
   }

   @Override
   public double getTurningOpenStepAngle()
   {
      double minimumHipOpeningAngle = Math.toRadians(10.0);
      double hipOpeningAngle = EuclidCoreTools.interpolate(minimumHipOpeningAngle,
                                                           steppingParameters.getMaxAngleTurnOutwards(),
                                                           teleoperationParameters.getTurnAggressiveness());
      return hipOpeningAngle;
   }

   @Override
   public double getTurningCloseStepAngle()
   {
      double hipClosingAngle = -teleoperationParameters.getTurnAggressiveness() * steppingParameters.getMaxAngleTurnInwards();
      return hipClosingAngle;
   }

   @Override
   public double getTurningStepWidth()
   {
      return steppingParameters.getTurningStepWidth();
   }

   public FootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public FramePose3D getGoalPose()
   {
      return goalPose;
   }

   public FramePose3D getLeftGoalFootPose()
   {
      return leftGoalFootPose;
   }

   public FramePose3D getRightGoalFootPose()
   {
      return rightGoalFootPose;
   }
}
