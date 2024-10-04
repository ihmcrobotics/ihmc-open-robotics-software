package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRDragData;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.interaction.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Random;

import static us.ihmc.rdx.ui.gizmo.RDXPathControlRingCollisionSelection.*;

/**
 * A gizmo designed for specifying the walk goals of bipedal robots.
 * It is able to be manipulated by mouse and keyboard by clicking and
 * dragging it around in the 3D scene as well as with VR controllers by
 * using the front triggers and dragging. The control ring is not able
 * to be pitched or rolled.
 *
 * It has arrows that can be used for various purposes.
 *
 */
public class RDXPathControlRingGizmo implements RenderableProvider
{
   public static final Color DISC_COLOR = RDXGizmoTools.CENTER_DEFAULT_COLOR;
   public static final Color X_ARROW_COLOR = RDXGizmoTools.X_AXIS_DEFAULT_COLOR;
   public static final Color Y_ARROW_COLOR = RDXGizmoTools.Y_AXIS_DEFAULT_COLOR;
   private static final double QUARTER_TURN = Math.PI / 2.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat discOuterRadius = new ImFloat(0.426f);
   private final ImFloat discInnerRadius = new ImFloat(0.290f);
   private final ImFloat discThickness = new ImFloat(0.03f);
   private final ImFloat arrowWidth = new ImFloat(0.257f);
   private final ImFloat arrowHeight = new ImFloat(0.137f);
   private final ImFloat arrowSpacing = new ImFloat(0.079f);
   private final ImFloat arrowTailWidthRatio = new ImFloat(0.5f);
   private final ImFloat arrowTailLengthRatio = new ImFloat(1.0f);
   private Material normalMaterial;
   private Material highlightedMaterial;
   private final DynamicLibGDXModel discModel = new DynamicLibGDXModel();
   private final DynamicLibGDXModel positiveXArrowModel = new DynamicLibGDXModel();
   private final DynamicLibGDXModel positiveYArrowModel = new DynamicLibGDXModel();
   private final DynamicLibGDXModel negativeXArrowModel = new DynamicLibGDXModel();
   private final DynamicLibGDXModel negativeYArrowModel = new DynamicLibGDXModel();
   private final RigidBodyTransform xArrowTailTransform = new RigidBodyTransform();
   private final RigidBodyTransform yArrowTailTransform = new RigidBodyTransform();
   private final RigidBodyTransform temporaryTailTransform = new RigidBodyTransform();
   private final Point3D closestCollision = new Point3D();
   private RDXPathControlRingCollisionSelection closestCollisionSelection = null;
   private double closestCollisionDistance;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean isGizmoHovered = false;
   private boolean isBeingManipulated = false;
   private final HollowCylinderRayIntersection hollowCylinderIntersection = new HollowCylinderRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveXArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveYArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final BoxRayIntersection negativeXArrowIntersection = new BoxRayIntersection();
   private final BoxRayIntersection negativeYArrowIntersection = new BoxRayIntersection();
   /** The main, source, true, base transform that this thing represents. */
   private RigidBodyTransform transformToParent;
   /** This pose 3D should always be left in world frame and represent this gizmo's pose. */
   private final FramePose3D framePose3D = new FramePose3D();
   private ReferenceFrame gizmoFrame;
   /** Gizmo transform to world so it can be calculated once. */
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   /** For being able to tell if the user moved the gizmo. */
   private final Notification gizmoModifiedByUser = new Notification();
   private RDXFocusBasedCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private final Plane3DMouseDragAlgorithm planeDragAlgorithm = new Plane3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private boolean showArrows = true;
   private boolean highlightingEnabled = true;
   private final double translateSpeedFactor = 0.5;
   private boolean queuePopupToOpen = false;
   private final Random random = new Random();
   private boolean proportionsNeedUpdate = false;
   private FrameBasedGizmoModification frameBasedGizmoModification;
   private final SideDependentList<RDXVRPickResult> vrPickResult = new SideDependentList<>(RDXVRPickResult::new);
   private final SideDependentList<Boolean> isGizmoHoveredVR = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> isRingBeingDraggedVR = new SideDependentList<>(false, false);
   private final SideDependentList<RDXPathControlRingCollisionSelection> closestVRCollisionSelection = new SideDependentList<>(null, null);

   public RDXPathControlRingGizmo()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public RDXPathControlRingGizmo(ReferenceFrame parentReferenceFrame)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      ReferenceFrame gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
      initialize(gizmoFrame, transformToParent);
   }

   public RDXPathControlRingGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public RDXPathControlRingGizmo(RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame)
   {
      ReferenceFrame gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame,
                                                                                                         gizmoTransformToParentFrameToModify);
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   private void initialize(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this.transformToParent = gizmoTransformToParentFrameToModify;
      this.gizmoFrame = gizmoFrame;

      RDXBaseUI.getInstance().getKeyBindings().register("Move control ring away from camera", "Up arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Move control ring toward from camera", "Down arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Move control ring left", "Left arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Move control ring right", "Right arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Drag control ring", "Left mouse");
      RDXBaseUI.getInstance().getKeyBindings().register("Drag control ring (yaw)", "Right mouse");
      RDXBaseUI.getInstance().getKeyBindings().register("Move control ring slowly", "Shift");

      for (RobotSide side : RobotSide.values)
         vrPickResult.get(side).setPickedObjectID(this, "Path Control Ring Gizmo");
   }

   public void setGizmoFrame(ReferenceFrame gizmoFrame)
   {
      this.gizmoFrame = gizmoFrame;
   }

   /**
    * Use of this method is assuming that this Gizmo is the owner of this frame
    * and not based on a frame managed externally.
    */
   public void setParentFrame(ReferenceFrame parentReferenceFrame)
   {
      gizmoFrame.remove();
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      create(baseUI.getPrimary3DPanel());
      baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
      baseUI.getVRManager().getContext().addVRPickCalculator(this::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
      baseUI.getPrimary3DPanel().getScene().addRenderableProvider(this, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDX3DPanel panel3D)
   {
      camera3D = panel3D.getCamera3D();
      boolean yawOnly = true;
      frameBasedGizmoModification = new FrameBasedGizmoModification(this::getGizmoFrame, () -> gizmoFrame.getParent(), camera3D, yawOnly);
      panel3D.addImGuiOverlayAddition(this::renderTooltipAndContextMenu);

      normalMaterial = createAlphaPaletteMaterial(RDXGizmoTools.X_AXIS_DEFAULT_COLOR.a);
      highlightedMaterial = createAlphaPaletteMaterial(RDXGizmoTools.X_AXIS_SELECTED_DEFAULT_COLOR.a);
      discModel.setMesh(meshBuilder ->
      {
         meshBuilder.addHollowCylinder(discThickness.get(),
                                       discOuterRadius.get(),
                                       discInnerRadius.get(),
                                       new Point3D(0.0, 0.0, 0.0),
                                       DISC_COLOR);
      });
      positiveXArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(discOuterRadius.get() + arrowSpacing.get(), 0.0, discThickness.get() / 2.0),
                                                 new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN),
                                                 X_ARROW_COLOR);
      });
      positiveYArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(0.0, discOuterRadius.get() + arrowSpacing.get(), discThickness.get() / 2.0),
                                                 new YawPitchRoll(0.0, 0.0, -QUARTER_TURN),
                                                 Y_ARROW_COLOR);
      });
      negativeXArrowModel.setMesh(meshBuilder ->
      {
         float arrowLength = arrowTailLengthRatio.get() * arrowHeight.get();
         xArrowTailTransform.getTranslation().set(-discOuterRadius.get() - arrowSpacing.get() - (arrowLength / 2.0), 0.0, discThickness.get() / 2.0);
         xArrowTailTransform.getRotation().setYawPitchRoll(QUARTER_TURN, 0.0, 0.0);
         meshBuilder.addBox(arrowTailWidthRatio.get() * arrowWidth.get(),
                            arrowLength,
                            discThickness.get(),
                            xArrowTailTransform.getTranslation(),
                            xArrowTailTransform.getRotation(),
                            X_ARROW_COLOR);
      });
      negativeYArrowModel.setMesh(meshBuilder ->
      {
         float arrowLength = arrowTailLengthRatio.get() * arrowHeight.get();
         yArrowTailTransform.getTranslation().set(0.0, -discOuterRadius.get() - arrowSpacing.get() - (arrowLength / 2.0), discThickness.get() / 2.0);
         yArrowTailTransform.getRotation().setYawPitchRoll(0.0, 0.0, 0.0);
         meshBuilder.addBox(arrowTailWidthRatio.get() * arrowWidth.get(),
                            arrowLength,
                            discThickness.get(),
                            yArrowTailTransform.getTranslation(),
                            yArrowTailTransform.getRotation(),
                            Y_ARROW_COLOR);
      });

      recreateGraphics();
   }

   private Material createAlphaPaletteMaterial(float alpha)
   {
      Material material = new Material();
      material.set(PBRTextureAttribute.createBaseColorTexture(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      material.set(new BlendingAttribute(true, alpha));
      return material;
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            if (!controller.getTriggerDragData().isDraggingSomething())
            {
               Line3DReadOnly pickRay = controller.getPickRay();
               if (!isRingBeingDraggedVR.get(side.getOppositeSide()))
                  closestVRCollisionSelection.put(side, determineCurrentSelectionFromPickRay(pickRay));
            }
            if (closestVRCollisionSelection.get(side) != null)
            {
               vrPickResult.get(side).setPointingAtCollision(closestCollisionDistance);
               controller.addPickResult(vrPickResult.get(side));
            }
         });
      }
   }

   private void calculateHovered(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         isGizmoHoveredVR.put(side, vrContext.getController(side).getSelectedPick() == vrPickResult.get(side));
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      calculateHovered(vrContext);
      processVRInputModification(vrContext);
   }

   private void processVRInputModification(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            RDXVRDragData triggerDragData = controller.getTriggerDragData();

            if (closestVRCollisionSelection.get(side) == RING)
            {
               if (triggerDragData.getDragJustStarted())
               {
                  triggerDragData.setObjectBeingDragged(this);
                  triggerDragData.setZUpDragStart(gizmoFrame);
               }
            }

            isRingBeingDraggedVR.put(side, triggerDragData.isBeingDragged(this));
            if (isRingBeingDraggedVR.get(side))
            {
               Line3DReadOnly pickRay = controller.getPickRay();

               if (triggerDragData.isDraggingSomething() && closestVRCollisionSelection.get(side) == RING)
               {
                  Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
                  frameBasedGizmoModification.translateInWorld(planarMotion);
                  closestCollision.add(planarMotion);
                  triggerDragData.updateZUpDrag(gizmoFrame);
                  double deltaYaw = triggerDragData.getZUpDragPose().getOrientation().getYaw() - gizmoFrame.getTransformToRoot().getRotation().getYaw();
                  frameBasedGizmoModification.yawInWorld(deltaYaw);
               }
               frameBasedGizmoModification.setAdjustmentNeedsToBeApplied();
            }
            else if (controller.getTriggerClickReleasedWithoutDrag())
            {
               if (closestVRCollisionSelection.get(side) == NEGATIVE_X_ARROW)
               {
                  frameBasedGizmoModification.yawInWorld(Math.PI);
               }
               else if (closestVRCollisionSelection.get(side) == POSITIVE_Y_ARROW)
               {
                  frameBasedGizmoModification.yawInWorld(Math.PI / 2.0);
               }
               else if (closestVRCollisionSelection.get(side) == NEGATIVE_Y_ARROW)
               {
                  frameBasedGizmoModification.yawInWorld(-Math.PI / 2.0);
               }
               frameBasedGizmoModification.setAdjustmentNeedsToBeApplied();
            }
         });
      }

      // after things have been modified, update the derivative stuff
      update();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(ImGuiMouseButton.Right);

      if (isWindowHovered)
      {
         if (!translateDragData.isDragging() && !yawDragData.isDragging())
         {
            Line3DReadOnly pickRay = input.getPickRayInWorld();
            closestCollisionSelection = determineCurrentSelectionFromPickRay(pickRay);
         }

         if (closestCollisionSelection != null)
         {
            pickResult.setDistanceToCamera(closestCollisionDistance);
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      calculateHovered(input);
      process3DViewInputModification(input);
   }

   /**
    * We separate the two so we can make decisions based on hover and clicks
    * before we start allowing modification of the gizmo.
    */
   void calculateHovered(ImGui3DViewInput input)
   {
      isGizmoHovered = input.isWindowHovered() && pickResult == input.getClosestPick();
   }

   void process3DViewInputModification(ImGui3DViewInput input)
   {
      boolean isWindowHovered = input.isWindowHovered();
      int yawMouseButton = ImGuiMouseButton.Right;
      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(yawMouseButton);

      if (isGizmoHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         queuePopupToOpen = true;
      }

      boolean isRingHovered = isGizmoHovered && closestCollisionSelection == RING;
      if (isRingHovered)
      {
         if (yawDragData.getDragJustStarted())
         {
            clockFaceDragAlgorithm.reset();
            yawDragData.setObjectBeingDragged(this);
         }
         else if (translateDragData.getDragJustStarted())
         {
            translateDragData.setObjectBeingDragged(this);
         }
      }
      isBeingManipulated = translateDragData.isBeingDragged(this) || yawDragData.isBeingDragged(this);

      if (isBeingManipulated)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (translateDragData.isDragging())
         {
            Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
            frameBasedGizmoModification.translateInWorld(planarMotion);
            closestCollision.add(planarMotion);
         }
         else // yaw dragging
         {
            if (clockFaceDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z, transformToWorld))
            {
               frameBasedGizmoModification.rotateInWorld(clockFaceDragAlgorithm.getMotion());
            }
         }
      }

      if (isWindowHovered)
      {
         // Use mouse wheel to yaw when ctrl key is held
         if (ImGui.getIO().getKeyCtrl() && input.getMouseWheelDelta() != 0.0f)
         {
            float deltaScroll = input.getMouseWheelDelta();
            // Add some noise to not get stuck in discrete space
            double noise = random.nextDouble() * 0.005;
            double speed = 0.012 + noise;
            frameBasedGizmoModification.yawInWorld(Math.signum(deltaScroll) * speed * Math.PI);
         }

         // keyboard based controls
         boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
         boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
         boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
         boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
         boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
         if (anyArrowHeld)
         {
            boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
            boolean altHeld = ImGui.getIO().getKeyAlt();
            boolean shiftHeld = ImGui.getIO().getKeyShift();
            double deltaTime = Gdx.graphics.getDeltaTime();

            if (altHeld) // orientation
            {
               double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
               Orientation3DBasics orientationToAdjust = frameBasedGizmoModification.beforeForRotationAdjustment();
               if (leftArrowHeld) // yaw +
               {
                  orientationToAdjust.appendYawRotation(amount);
               }
               if (rightArrowHeld) // yaw -
               {
                  orientationToAdjust.appendYawRotation(-amount);
               }
               frameBasedGizmoModification.afterRotationAdjustment(FrameBasedGizmoModification.PREPEND);
            }
            else // translation
            {
               double amount = deltaTime * (shiftHeld ? 0.05 : 0.4);
               Point3DBasics positionToAdjust = frameBasedGizmoModification.beforeForTranslationAdjustment();
               if (upArrowHeld && !ctrlHeld) // x +
               {
                  positionToAdjust.addX(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && !ctrlHeld) // x -
               {
                  positionToAdjust.subX(getTranslateSpeedFactor() * amount);
               }
               if (leftArrowHeld) // y +
               {
                  positionToAdjust.addY(getTranslateSpeedFactor() * amount);
               }
               if (rightArrowHeld) // y -
               {
                  positionToAdjust.subY(getTranslateSpeedFactor() * amount);
               }
               if (upArrowHeld && ctrlHeld) // z +
               {
                  positionToAdjust.addZ(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && ctrlHeld) // z -
               {
                  positionToAdjust.subZ(getTranslateSpeedFactor() * amount);
               }
            }

            frameBasedGizmoModification.setAdjustmentNeedsToBeApplied();
         }
      }

      // after things have been modified, update the derivative stuff
      update();
   }

   private void renderTooltipAndContextMenu()
   {
      if (queuePopupToOpen)
      {
         queuePopupToOpen = false;

         ImGui.openPopup(labels.get("Popup"));
      }

      if (ImGui.beginPopup(labels.get("Popup")))
      {
         renderImGuiTuner();
         if (ImGui.menuItem("Close"))
            ImGui.closeCurrentPopup();
         ImGui.endPopup();
      }
   }

   /** Call this instead of calculate3DViewPick and process3DViewInput if the gizmo is deactivated. */
   public void update()
   {
      if (frameBasedGizmoModification.applyAdjustmentIfNeeded(transformToParent))
      {
         gizmoModifiedByUser.set();
      }

      gizmoFrame.update();
      frameBasedGizmoModification.setToZeroGizmoFrame();

      framePose3D.setToZero(gizmoFrame);
      framePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      framePose3D.get(transformToWorld);
      updateGraphicTransforms();
      LibGDXTools.toEuclid(camera3D.position, cameraPosition);
      distanceToCamera = cameraPosition.distance(framePose3D.getPosition());

      if (proportionsNeedUpdate)
      {
         proportionsNeedUpdate = false;
         recreateGraphics();
      }
   }

   private void updateGraphicTransforms()
   {
      LibGDXTools.toLibGDX(transformToWorld, discModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(transformToWorld, positiveXArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(transformToWorld, positiveYArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(transformToWorld, negativeXArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(transformToWorld, negativeYArrowModel.getOrCreateModelInstance().transform);
   }

   private RDXPathControlRingCollisionSelection determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      RDXPathControlRingCollisionSelection closestCollisionSelection = null;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      hollowCylinderIntersection.update(discThickness.get(), discOuterRadius.get(), discInnerRadius.get(), discThickness.get() / 2.0, transformToWorld);
      double distance = hollowCylinderIntersection.intersect(pickRay);
      if (!Double.isNaN(distance) && distance < closestCollisionDistance)
      {
         closestCollisionDistance = distance;
         closestCollisionSelection = RING;
         closestCollision.set(hollowCylinderIntersection.getClosestIntersection());
      }
      if (showArrows)
      {
         positiveXArrowIntersection.update(arrowWidth.get(),
                                           arrowHeight.get(),
                                           discThickness.get(),
                                           new Point3D(discOuterRadius.get() + arrowSpacing.get(), 0.0, discThickness.get() / 2.0),
                                           new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN), transformToWorld);
         distance = positiveXArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = POSITIVE_X_ARROW;
            closestCollision.set(positiveXArrowIntersection.getClosestIntersection());
         }
         positiveYArrowIntersection.update(arrowWidth.get(),
                                           arrowHeight.get(),
                                           discThickness.get(),
                                           new Point3D(0.0, discOuterRadius.get() + arrowSpacing.get(), discThickness.get() / 2.0),
                                           new YawPitchRoll(0.0, 0.0, -QUARTER_TURN), transformToWorld);
         distance = positiveYArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = POSITIVE_Y_ARROW;
            closestCollision.set(positiveYArrowIntersection.getClosestIntersection());
         }
         temporaryTailTransform.set(xArrowTailTransform);
         transformToWorld.transform(temporaryTailTransform);
         boolean intersects = negativeXArrowIntersection.intersect(arrowTailWidthRatio.get() * arrowWidth.get(),
                                                                   arrowTailLengthRatio.get() * arrowHeight.get(),
                                                                   discThickness.get(),
                                                                   temporaryTailTransform,
                                                                   pickRay);
         distance = negativeXArrowIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (intersects && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = NEGATIVE_X_ARROW;
            closestCollision.set(negativeXArrowIntersection.getFirstIntersectionToPack());
         }
         temporaryTailTransform.set(yArrowTailTransform);
         transformToWorld.transform(temporaryTailTransform);
         intersects = negativeYArrowIntersection.intersect(arrowTailWidthRatio.get() * arrowWidth.get(),
                                                           arrowTailLengthRatio.get() * arrowHeight.get(),
                                                           discThickness.get(),
                                                           temporaryTailTransform,
                                                           pickRay);
         distance = negativeYArrowIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (intersects && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = NEGATIVE_Y_ARROW;
            closestCollision.set(negativeYArrowIntersection.getFirstIntersectionToPack());
         }
      }

      updateMaterialHighlighting();

      return closestCollisionSelection;
   }

   private void updateMaterialHighlighting()
   {
      boolean gizmoHoveredByVR = isGizmoHoveredVR.get(RobotSide.LEFT) || isGizmoHoveredVR.get(RobotSide.RIGHT);
      boolean gizmoHoveredByAnything = isGizmoHovered || gizmoHoveredByVR;
      boolean highlightingPrior = highlightingEnabled && gizmoHoveredByAnything;
      discModel.setMaterial(highlightingPrior && (closestCollisionSelection == RING || closestVRCollisionSelection.get(RobotSide.LEFT) == RING ||
                                                  closestVRCollisionSelection.get(RobotSide.RIGHT) == RING)? highlightedMaterial : normalMaterial);
      positiveXArrowModel.setMaterial(highlightingPrior &&
                                      (closestCollisionSelection == POSITIVE_X_ARROW  ||
                                       closestVRCollisionSelection.get(RobotSide.LEFT) == POSITIVE_X_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.RIGHT) == POSITIVE_X_ARROW)? highlightedMaterial : normalMaterial);
      positiveYArrowModel.setMaterial(highlightingPrior &&
                                      (closestCollisionSelection == POSITIVE_Y_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.LEFT) == POSITIVE_Y_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.RIGHT) == POSITIVE_Y_ARROW)? highlightedMaterial : normalMaterial);
      negativeXArrowModel.setMaterial(highlightingPrior &&
                                      (closestCollisionSelection == NEGATIVE_X_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.LEFT) == NEGATIVE_X_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.RIGHT) == NEGATIVE_X_ARROW)? highlightedMaterial : normalMaterial);
      negativeYArrowModel.setMaterial(highlightingPrior &&
                                      (closestCollisionSelection == NEGATIVE_Y_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.LEFT) == NEGATIVE_Y_ARROW ||
                                       closestVRCollisionSelection.get(RobotSide.RIGHT) == NEGATIVE_Y_ARROW)? highlightedMaterial : normalMaterial);
   }

   public RDXPanel createTunerPanel(String name)
   {
      return new RDXPanel("Footstep Ring Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
   }

   public void renderImGuiTuner()
   {
      frameBasedGizmoModification.renderImGuiTunerWidgets();

      if (ImGui.collapsingHeader(labels.get("Controls")))
      {
         ImGui.text("Use the left mouse drag on the ring to move the gizmo around on its X-Y plane.");
         ImGui.text("Use the right mouse drag on the ring to adjust the yaw.");
         // TODO: Add keyboard & scroll wheel controls
      }

      if (ImGui.button(labels.get("Reset")))
      {
         transformToParent.setToZero();
      }

      if (ImGui.collapsingHeader(labels.get("Visual options")))
      {
         ImGui.pushItemWidth(100.00f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Disc outer radius"), discOuterRadius.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Disc inner radius"), discInnerRadius.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Disc thickness"), discThickness.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow width"), arrowWidth.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow height"), arrowHeight.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow spacing"), arrowSpacing.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow tail width ratio"), arrowTailWidthRatio.getData(), 0.001f, 0.0f, 1000.0f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow tail length ratio"), arrowTailLengthRatio.getData(), 0.001f, 0.0f, 1000.0f);
         ImGui.popItemWidth();
      }
   }

   private void recreateGraphics()
   {
      updateMaterialHighlighting();
      discModel.invalidateMesh();
      positiveXArrowModel.invalidateMesh();
      positiveYArrowModel.invalidateMesh();
      negativeXArrowModel.invalidateMesh();
      negativeYArrowModel.invalidateMesh();
      updateGraphicTransforms();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      discModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      if (showArrows)
      {
         positiveXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         positiveYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         negativeXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         negativeYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      }
   }

   public Pose3DReadOnly getPose3D()
   {
      return framePose3D;
   }

   // TODO: Make this transform the ground truth and give the pose as needed only
   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public ReferenceFrame getGizmoFrame()
   {
      return gizmoFrame;
   }

   public boolean getAnyPartHovered()
   {
      return isGizmoHovered;
   }

   public boolean getAnyArrowHovered()
   {
      boolean anyArrowHovered = closestCollisionSelection == POSITIVE_X_ARROW;
      anyArrowHovered |= closestCollisionSelection == POSITIVE_Y_ARROW;
      anyArrowHovered |= closestCollisionSelection == NEGATIVE_X_ARROW;
      anyArrowHovered |= closestCollisionSelection == NEGATIVE_Y_ARROW;
      return isGizmoHovered && anyArrowHovered;
   }

   public boolean getRingHovered()
   {
      return isGizmoHovered && closestCollisionSelection == RING;
   }

   public boolean getPositiveXArrowHovered()
   {
      return isGizmoHovered && closestCollisionSelection == POSITIVE_X_ARROW ||
             (isGizmoHoveredVR.get(RobotSide.LEFT) && closestVRCollisionSelection.get(RobotSide.LEFT) == POSITIVE_X_ARROW)||
             (isGizmoHoveredVR.get(RobotSide.RIGHT) && closestVRCollisionSelection.get(RobotSide.RIGHT) == POSITIVE_X_ARROW);
   }

   public boolean getPositiveYArrowHovered()
   {
      return isGizmoHovered && closestCollisionSelection == POSITIVE_Y_ARROW ||
             (isGizmoHoveredVR.get(RobotSide.LEFT) && closestVRCollisionSelection.get(RobotSide.LEFT) == POSITIVE_Y_ARROW) ||
             (isGizmoHoveredVR.get(RobotSide.RIGHT) && closestVRCollisionSelection.get(RobotSide.RIGHT) == POSITIVE_Y_ARROW);
   }

   public boolean getNegativeXArrowHovered()
   {
      return isGizmoHovered && closestCollisionSelection == NEGATIVE_X_ARROW ||
             (isGizmoHoveredVR.get(RobotSide.LEFT) && closestVRCollisionSelection.get(RobotSide.LEFT) == NEGATIVE_X_ARROW) ||
             (isGizmoHoveredVR.get(RobotSide.RIGHT) && closestVRCollisionSelection.get(RobotSide.RIGHT) == NEGATIVE_X_ARROW);
   }

   public boolean getNegativeYArrowHovered()
   {
      return isGizmoHovered && closestCollisionSelection == NEGATIVE_Y_ARROW ||
             (isGizmoHoveredVR.get(RobotSide.LEFT) && closestVRCollisionSelection.get(RobotSide.LEFT) == NEGATIVE_Y_ARROW) ||
             (isGizmoHoveredVR.get(RobotSide.RIGHT) && closestVRCollisionSelection.get(RobotSide.RIGHT) == NEGATIVE_Y_ARROW);
   }

   public void setShowArrows(boolean showArrows)
   {
      this.showArrows = showArrows;
   }

   private double getTranslateSpeedFactor()
   {
      return translateSpeedFactor * distanceToCamera;
   }

   public void setHighlightingEnabled(boolean highlightingEnabled)
   {
      this.highlightingEnabled = highlightingEnabled;
   }

   public Notification getGizmoModifiedByUser()
   {
      return gizmoModifiedByUser;
   }

   public SideDependentList<Boolean> getIsRingBeingDraggedVR()
   {
      return isRingBeingDraggedVR;
   }

   public SideDependentList<Boolean> getIsGizmoHoveredVR()
   {
      return isGizmoHoveredVR;
   }
}
