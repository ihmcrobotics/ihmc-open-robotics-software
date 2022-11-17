package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPathControlRingGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.PathTypeStepParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

public class RDXWalkPathControlRing implements PathTypeStepParameters
{
   private final RDXPathControlRingGizmo footstepPlannerGoalGizmo = new RDXPathControlRingGizmo();
   private boolean selectedMouse = false;
   private boolean modifiedMouse = false;
   private boolean newlyModifiedMouse = false;
   private boolean ringPickSelectedMouse;
   private RDX3DPanel panel3D;
   private RDXTeleoperationParameters teleoperationParameters;
   private MovingReferenceFrame midFeetZUpFrame;
   private ResettableExceptionHandlingExecutorService footstepPlanningThread;
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private RDXFootstepGraphic leftStanceFootstepGraphic;
   private RDXFootstepGraphic rightStanceFootstepGraphic;
   private RDXFootstepGraphic leftGoalFootstepGraphic;
   private RDXFootstepGraphic rightGoalFootstepGraphic;
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
   private RDXFootstepPlanGraphic foostepPlanGraphic;
   private double halfIdealFootstepWidth;
   private volatile FootstepPlan footstepPlan;
   private volatile FootstepPlan footstepPlanToGenerateMeshes;
   private final AxisAngle walkFacingDirection = new AxisAngle();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXFootstepPlanningAlgorithm footstepPlanningAlgorithm = RDXFootstepPlanningAlgorithm.A_STAR;
   private TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final FootstepPlannerGoal turnWalkTurnGoal = new FootstepPlannerGoal();
   private TurnStraightTurnFootstepGenerator turnStraightTurnFootstepGenerator;
   private SimplePathParameters turnStraightTurnParameters;
   private SteppingParameters steppingParameters;
   private RDXWalkPathType walkPathType = RDXWalkPathType.STRAIGHT;
   private ImGui3DViewInput latestInput;

   private boolean selectedVR = false;
   private boolean modifiedVR = false;
   private boolean newlyModifiedVR = false;
   private boolean ringPickSelectedVR;

   private boolean selected = false;
   private boolean modified = false;
   private boolean newlyModified = false;

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXTeleoperationParameters teleoperationParameters,
                      RDXVRContext context)
   {
      this.panel3D = panel3D;
      this.teleoperationParameters = teleoperationParameters;
      footstepPlannerGoalGizmo.create(panel3D.getCamera3D(), context);
      panel3D.addImGuiOverlayAddition(this::renderTooltips);
      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      footFrames = syncedRobot.getReferenceFrames().getSoleFrames();

      // periodic thread to footstep plan
      footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("WalkPathControlPlanning", true, 1);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      steppingParameters = robotModel.getWalkingControllerParameters().getSteppingParameters();
      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      leftStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightStanceFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);

      goalFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("goalPose",
                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                  footstepPlannerGoalGizmo.getTransformToParent());

      turnWalkTurnPlanner = new TurnWalkTurnPlanner(footstepPlannerParameters);

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      foostepPlanGraphic = new RDXFootstepPlanGraphic(contactPoints);
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
   }

   public void update(RDXInteractableFootstepPlan plannedFootstepPlacement)
   {
      if (!modifiedMouse)
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
         if (footstepPlannerGoalGizmo.isSendSteps())
         {
            plannedFootstepPlacement.walkFromSteps();
            footstepPlannerGoalGizmo.setSendSteps(false);
         }
      }
      foostepPlanGraphic.update();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.calculate3DViewPick(input);
   }

   // This happens after update. (Currently this also queries for VRStuff from footstepPlannerGoalGizmo.)
   public void process3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;
      boolean leftMouseReleasedWithoutDrag = input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      footstepPlannerGoalGizmo.process3DViewInput(input, selectedMouse);
      ringPickSelectedMouse = footstepPlannerGoalGizmo.getHollowCylinderPickSelectedMouse();

      if (ringPickSelectedMouse && leftMouseReleasedWithoutDrag)
      {
         becomeModifiedMouse(true);
      }
      if (selectedMouse && !footstepPlannerGoalGizmo.getAnyPartPickSelectedMouse() && leftMouseReleasedWithoutDrag)
      {
         selectedMouse = false;
      }

      if (modifiedMouse)
      {
         updateStuff();
      }
      if (selectedMouse && leftMouseReleasedWithoutDrag)
      {
         updateFromArrowPick(footstepPlannerGoalGizmo.getPositiveXArrowPickSelectedMouse(),
                             footstepPlannerGoalGizmo.getPositiveYArrowPickSelectedMouse(),
                             footstepPlannerGoalGizmo.getNegativeXArrowPickSelectedMouse(),
                             footstepPlannerGoalGizmo.getNegativeYArrowPickSelectedMouse(),
                             footstepPlannerGoalGizmo.getAnyArrowPickSelectedMouse());
      }
      if (selectedMouse && footstepPlannerGoalGizmo.isNewlyModifiedMouse())
      {
         queueFootstepPlan();
      }

      if (modifiedMouse && selectedMouse && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         delete();
      }
      if (selectedMouse && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         selectedMouse = false;
      }

      // combine changes from mouse and vr
      combineChanges();
      footstepPlannerGoalGizmo.setShowArrows(selected);
      footstepPlannerGoalGizmo.setHighlightingEnabled(modified);
   }

   private void combineChanges()
   {
      selected = selectedMouse || selectedVR;
      modified = modifiedMouse || modifiedVR;
      newlyModified = newlyModifiedMouse || newlyModifiedVR;
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.calculateVRPick(vrContext);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      footstepPlannerGoalGizmo.processVRInput(vrContext);
      boolean isVRTriggerPressed = footstepPlannerGoalGizmo.isVRTriggerPressed();
      ringPickSelectedVR = footstepPlannerGoalGizmo.getHollowCylinderPickSelectedVR();

      if (ringPickSelectedVR && isVRTriggerPressed)
      {
         becomeModifiedVR(true);
      }
      if (selectedVR && !footstepPlannerGoalGizmo.getAnyPartPickSelectedVR() && isVRTriggerPressed)
      {
         selectedVR = false;
      }

      if (modifiedVR)
      {
         updateStuff();
      }
      if (selectedVR && isVRTriggerPressed)
      {
         updateFromArrowPick(footstepPlannerGoalGizmo.getPositiveXArrowPickSelectedVR(),
                             footstepPlannerGoalGizmo.getPositiveYArrowPickSelectedVR(),
                             footstepPlannerGoalGizmo.getNegativeXArrowPickSelectedVR(),
                             footstepPlannerGoalGizmo.getNegativeYArrowPickSelectedVR(),
                             footstepPlannerGoalGizmo.getAnyArrowPickSelectedVR());
      }

      if (selectedVR && footstepPlannerGoalGizmo.isNewlyModifiedVR())
      {
         queueFootstepPlan();
      }

      /* TODO: assign another key for these in vr version?
      if (modifiedVR && selectedVR && SOMEKEY PRESS FROM JOYSTICK)
      {
         delete();
      }
      if (selectedVR && footstepPlannerGoalGizmo.isNewlyModifiedVR())
      {
         queueFootstepPlan();
      }

      if (modifiedVR && selected && ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         delete();
      }
      if (selectedVR && ImGui.isKeyReleased(ImGuiTools.getEscapeKey()))
      {
         selected = false;
      }

       */

      // combine changes from mouse and vr
      combineChanges();
      footstepPlannerGoalGizmo.setShowArrows(selectedVR);
      footstepPlannerGoalGizmo.setHighlightingEnabled(modifiedVR);
   }

   private void updateFromArrowPick(boolean positiveX, boolean positiveY, boolean negativeX, boolean negativeY, boolean anyArrowSelected)
   {
      if (positiveX)
      {
         walkPathType = RDXWalkPathType.STRAIGHT;
         walkFacingDirection.set(Axis3D.Z, 0.0);
      }
      else if (positiveY)
      {
         walkPathType = RDXWalkPathType.LEFT_SHUFFLE;
         walkFacingDirection.set(Axis3D.Z, Math.PI / 2.0);
      }
      else if (negativeX)
      {
         walkPathType = RDXWalkPathType.REVERSE;
         walkFacingDirection.set(Axis3D.Z, Math.PI);
      }
      else if (negativeY)
      {
         walkPathType = RDXWalkPathType.RIGHT_SHUFFLE;
         walkFacingDirection.set(Axis3D.Z, -Math.PI / 2.0);
      }
      if (anyArrowSelected)
      {
         footstepPlannerGoalGizmo.getTransformToParent().appendOrientation(walkFacingDirection);
         updateStuff();
         queueFootstepPlan();
      }
   }

   private void queueFootstepPlan()
   {
      footstepPlanningThread.clearQueueAndExecute(() ->
      {
         switch (footstepPlanningAlgorithm)
         {
            case A_STAR ->
            {
               planFoostepsUsingAStarPlanner(new Pose3D(leftStanceFootPose),
                                             new Pose3D(rightStanceFootPose),
                                             new Pose3D(leftGoalFootPose),
                                             new Pose3D(rightGoalFootPose));

            }
            case TURN_WALK_TURN ->
            {
               planFootstepsUsingTurnWalkTurnPlanner();
            }
            case TURN_STRAIGHT_TURN ->
            {
               planFootstepsUsingTurnStraightTurnFootstepGenerator();
            }
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
      footstepPlannerRequest.setSnapGoalSteps(false);
      footstepPlannerRequest.setRequestId(footstepPlannerId.getAndIncrement());
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      footstepPlan = footstepPlanToGenerateMeshes = new FootstepPlan(footstepPlannerOutput.getFootstepPlan());
   }

   public void renderImGuiWidgets()
   {
      footstepPlannerGoalGizmo.renderImGuiWidgets();
      if (ImGui.radioButton(labels.get("A* Planner"), footstepPlanningAlgorithm == RDXFootstepPlanningAlgorithm.A_STAR))
      {
         footstepPlanningAlgorithm = RDXFootstepPlanningAlgorithm.A_STAR;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Turn Walk Turn"), footstepPlanningAlgorithm == RDXFootstepPlanningAlgorithm.TURN_WALK_TURN))
      {
         footstepPlanningAlgorithm = RDXFootstepPlanningAlgorithm.TURN_WALK_TURN;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Turn Straight Turn"), footstepPlanningAlgorithm == RDXFootstepPlanningAlgorithm.TURN_STRAIGHT_TURN))
      {
         footstepPlanningAlgorithm = RDXFootstepPlanningAlgorithm.TURN_STRAIGHT_TURN;
      }

      ImGui.text("Control ring:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Deleted"), !selectedMouse && !modifiedMouse))
      {
         delete();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Modified"), !selectedMouse && modifiedMouse))
      {
         becomeModifiedMouse(false);
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Selected"), selectedMouse && modifiedMouse))
      {
         becomeModifiedMouse(true);
      }
   }

   public void becomeModifiedMouse(boolean selected)
   {
      this.selectedMouse = selected;
      if (!modifiedMouse)
      {
         modifiedMouse = true;
         newlyModifiedMouse = true;
         walkFacingDirection.set(Axis3D.Z, 0.0);
         updateStuff();
         queueFootstepPlan();
      }
   }

   public void becomeModifiedVR(boolean selected)
   {
      this.selectedVR = selected;
      if (!modifiedVR)
      {
         modifiedVR = true;
         newlyModifiedVR = true;
         walkFacingDirection.set(Axis3D.Z, 0.0);
         updateStuff();
         queueFootstepPlan();
      }
   }

   public boolean pollIsNewlyModified()
   {
      boolean newlyModifiedReturn = newlyModifiedMouse || newlyModifiedVR;
      newlyModifiedMouse = false;
      newlyModifiedVR = false;
      return newlyModifiedReturn;
   }

   private void renderTooltips()
   {
      if (selectedMouse && footstepPlannerGoalGizmo.isGizmoHoveredMouse())
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
      if (modifiedMouse)
      {
         leftStanceFootstepGraphic.getRenderables(renderables, pool);
         rightStanceFootstepGraphic.getRenderables(renderables, pool);
         leftGoalFootstepGraphic.getRenderables(renderables, pool);
         rightGoalFootstepGraphic.getRenderables(renderables, pool);
         foostepPlanGraphic.getRenderables(renderables, pool);
      }
      if (modifiedMouse || modifiedVR || ringPickSelectedMouse || ringPickSelectedVR || footstepPlannerGoalGizmo.isRingHoveredFromVR())
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
      }

      footstepPlannerGoalGizmo.getVRLineRenderable(renderables, pool);
   }

   public void delete()
   {
      selectedMouse = false;
      modifiedMouse = false;
      selectedVR = false;
      modifiedVR = false;
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

   public boolean isSelectedMouse()
   {
      return selectedMouse;
   }
}
