package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMeshBuilder;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.interaction.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.rdx.ui.gizmo.RDXGizmoTools.AXIS_COLORS;
import static us.ihmc.rdx.ui.gizmo.RDXGizmoTools.AXIS_SELECTED_COLORS;

/**
 * A gizmo for manipulating a Pose3D in 3D space using
 * mouse keyboard and monitor (KVM) by clicking and dragging on
 * the parts of it.
 *
 * TODO: Support VR interaction.
 */
public class RDXPose3DGizmo implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat torusRadius = new ImFloat(0.5f);
   private final ImFloat torusCameraSize = new ImFloat(0.067f);
   private final ImFloat torusTubeRadiusRatio = new ImFloat(0.074f);
   private final ImFloat centerSphereRadius = new ImFloat();
   private final ImFloat centerSphereToTorusRatio = new ImFloat(1.0f / 8.0f);
   private final ImFloat arrowLengthRatio = new ImFloat(0.431f);
   private final ImFloat arrowHeadBodyLengthRatio = new ImFloat(0.480f);
   private final ImFloat arrowHeadBodyRadiusRatio = new ImFloat(2.0f);
   private final ImFloat arrowSpacingFactor = new ImFloat(2.22f);
   private final ImBoolean resizeAutomatically = new ImBoolean(true);
   private double arrowBodyRadius;
   private double arrowLength;
   private double arrowBodyLength;
   private double arrowHeadRadius;
   private double arrowHeadLength;
   private double arrowSpacing;
   private final Material[] normalMaterials = new Material[3];
   private final Material[] highlightedMaterials = new Material[3];
   private final Axis3DRotations axisRotations = new Axis3DRotations();
   private final DynamicLibGDXModel centerSphereModel = new DynamicLibGDXModel();
   private final DynamicLibGDXModel[] arrowModels = new DynamicLibGDXModel[3];
   private final DynamicLibGDXModel[] torusModels = new DynamicLibGDXModel[3];
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection = null;
   private double closestCollisionDistance;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean isGizmoHovered = false;
   private boolean isBeingManipulated = false;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final SphereRayIntersection centerSphereIntersection = new SphereRayIntersection();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private final DiscreteArrowRayIntersection arrowIntersection = new DiscreteArrowRayIntersection();
   /** The main, source, true, base transform that this thing represents. */
   private RigidBodyTransform transformToParent;
   /** This pose 3D should always be left in world frame and represent this gizmo's pose. */
   private final FramePose3D framePose3D = new FramePose3D();
   private ReferenceFrame gizmoFrame;
   /** Gizmo transform to world so it can be calculated once. */
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   /** Transforms to world for placing the graphics. */
   private final RigidBodyTransform[] axisTransformToWorlds = new RigidBodyTransform[3];
   /** For being able to tell if the user moved the gizmo. */
   private final Notification gizmoModifiedByUser = new Notification();
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private final Plane3DMouseDragAlgorithm planeDragAlgorithm = new Plane3DMouseDragAlgorithm();
   private final Line3DMouseDragAlgorithm lineDragAlgorithm = new Line3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private RDXFocusBasedCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private double lastDistanceToCamera = -1.0;
   private final double translateSpeedFactor = 0.5;
   private boolean queuePopupToOpen = false;
   private boolean proportionsNeedUpdate = false;
   private FrameBasedGizmoModification frameBasedGizmoModification;

   public RDXPose3DGizmo()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public RDXPose3DGizmo(String gizmoFrameName)
   {
      this(gizmoFrameName, ReferenceFrame.getWorldFrame());
   }

   public RDXPose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      this(ReferenceFrameMissingTools.computeFrameName(), parentReferenceFrame);
   }

   public RDXPose3DGizmo(String gizmoFrameName, ReferenceFrame parentReferenceFrame)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      ReferenceFrame gizmoFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(gizmoFrameName,
                                                                                                  parentReferenceFrame,
                                                                                                  transformToParent);
      initialize(gizmoFrame, transformToParent);
   }

   public RDXPose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public RDXPose3DGizmo(RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame)
   {
      this(ReferenceFrameMissingTools.computeFrameName(), gizmoTransformToParentFrameToModify, parentReferenceFrame);
   }

   public RDXPose3DGizmo(String gizmoFrameName, RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame)
   {
      ReferenceFrame gizmoFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(gizmoFrameName,
                                                                                                  parentReferenceFrame,
                                                                                                  gizmoTransformToParentFrameToModify);
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   private void initialize(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this.transformToParent = gizmoTransformToParentFrameToModify;
      this.gizmoFrame = gizmoFrame;

      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment X+", "Up arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment X-", "Down arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment Y+", "Left arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment Y-", "Right arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment Z+", "Ctrl + Up arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Translation adjustment Z-", "Ctrl + Down arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Pitch adjustment +", "Alt + Up arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Pitch adjustment -", "Alt + Down arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Roll adjustment +", "Alt + Right arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Roll adjustment -", "Alt + Left arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Yaw adjustment +", "Ctrl + Left arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Yaw adjustment -", "Ctrl + Right arrow");
      RDXBaseUI.getInstance().getKeyBindings().register("Yaw adjustment +", "Ctrl + Mouse scroll down");
      RDXBaseUI.getInstance().getKeyBindings().register("Yaw adjustment -", "Ctrl + Mouse scroll up");
      RDXBaseUI.getInstance().getKeyBindings().register("Fine adjustment modifier", "Shift");
      RDXBaseUI.getInstance().getKeyBindings().register("Manipulate axes", "Left mouse drag");
      RDXBaseUI.getInstance().getKeyBindings().register("Open context menu", "Right mouse click");
      RDXBaseUI.getInstance().getKeyBindings().register("Open context menu", "Right mouse click");
   }

   public void setGizmoFrame(ReferenceFrame gizmoFrame)
   {
      this.gizmoFrame = gizmoFrame;
   }

   /**
    * Use of this method is assuming that this Gizmo is the owner of this frame
    * and not based on a frame managed externally.
    */
   public void setParentFrame(ReferenceFrame newParentFrame)
   {
      gizmoFrame.remove();
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(newParentFrame, transformToParent);
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame newParentFrame)
   {
      RigidBodyTransform newTransformToParent = new RigidBodyTransform();
      gizmoFrame.getTransformToDesiredFrame(newTransformToParent, newParentFrame);
      transformToParent.set(newTransformToParent);
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(newParentFrame, transformToParent);
   }

   public void createAndSetupDefault(RDX3DPanel panel3D)
   {
      create(panel3D);
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDX3DPanel panel3D)
   {
      camera3D = panel3D.getCamera3D();
      frameBasedGizmoModification = new FrameBasedGizmoModification(this::getGizmoFrame, () -> gizmoFrame.getParent(), camera3D);
      panel3D.addImGuiOverlayAddition(this::renderTooltipAndContextMenu);

      centerSphereModel.setMesh(meshBuilder -> meshBuilder.addSphere(centerSphereRadius.get(), RDXGizmoTools.CENTER_DEFAULT_COLOR));

      for (Axis3D axis : Axis3D.values)
      {
         Color color = AXIS_COLORS[axis.ordinal()];
         normalMaterials[axis.ordinal()] = new Material();
         normalMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
         normalMaterials[axis.ordinal()].set(new BlendingAttribute(true, color.a));
         highlightedMaterials[axis.ordinal()] = new Material();
         highlightedMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
         highlightedMaterials[axis.ordinal()].set(new BlendingAttribute(true, AXIS_SELECTED_COLORS[axis.ordinal()].a));

         arrowModels[axis.ordinal()] = new DynamicLibGDXModel();
         arrowModels[axis.ordinal()].setMesh(meshBuilder ->
         {
            // Euclid cylinders are defined from the center, but mesh builder defines them from the bottom
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing), color);
            meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing + arrowBodyLength), color);
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, -0.5 * arrowSpacing), FLIP_180, color);
         });
         torusModels[axis.ordinal()] = new DynamicLibGDXModel();
         int resolution = 25;
         torusModels[axis.ordinal()].setMesh(
            meshBuilder -> meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), resolution, color));
         axisTransformToWorlds[axis.ordinal()] = new RigidBodyTransform();
      }

      recreateGraphics();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      // Here we are trying to avoid unecessary computation in collision calculation by filtering out
      // some common scenarios where we don't need to calculate the pick, which can be expensive
      if (isWindowHovered && (!manipulationDragData.isDragging() || manipulationDragData.getDragJustStarted()))
      {
         // This part is happening when the user could presumably start a drag
         // on this gizmo at any time

         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (closestCollisionSelection != null)
         {
            pickResult.setDistanceToCamera(closestCollisionDistance);
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean isWindowHovered = input.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      isGizmoHovered = isWindowHovered && pickResult == input.getClosestPick();

      if (isGizmoHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         queuePopupToOpen = true;
      }

      if (isGizmoHovered && manipulationDragData.getDragJustStarted())
      {
         clockFaceDragAlgorithm.reset();
         manipulationDragData.setObjectBeingDragged(this);
      }
      isBeingManipulated = manipulationDragData.isBeingDragged(this);

      updateMaterialHighlighting();

      if (isBeingManipulated && closestCollisionSelection != null)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (closestCollisionSelection.isCenter())
         {
            Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
            frameBasedGizmoModification.translateInWorld(planarMotion);
            closestCollision.add(planarMotion);
         }
         else if (closestCollisionSelection.isLinear())
         {
            Vector3DReadOnly linearMotion = lineDragAlgorithm.calculate(pickRay,
                                                                        closestCollision,
                                                                        axisRotations.get(closestCollisionSelection.toAxis3D()),
                                                                        transformToWorld);
            frameBasedGizmoModification.translateInWorld(linearMotion);
            closestCollision.add(linearMotion);
         }
         else if (closestCollisionSelection.isAngular())
         {
            if (clockFaceDragAlgorithm.calculate(pickRay, closestCollision, axisRotations.get(closestCollisionSelection.toAxis3D()), transformToWorld))
            {
               frameBasedGizmoModification.rotateInWorld(clockFaceDragAlgorithm.getMotion());
            }
         }
      }

      if (isWindowHovered)
      {
         boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
         boolean shiftHeld = ImGui.getIO().getKeyShift();

         // Use mouse wheel to yaw when ctrl key is held
         if (ctrlHeld && input.getMouseWheelDelta() != 0.0f)
         {
            float deltaScroll = input.getMouseWheelDelta();
            double amount = shiftHeld ? 0.001 : 0.012;
            frameBasedGizmoModification.yawInWorld(Math.signum(deltaScroll) * amount * Math.PI);
         }

         // keyboard based controls
         boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
         boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
         boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
         boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
         boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
         if (anyArrowHeld) // only the arrow keys do the moving
         {
            boolean altHeld = ImGui.getIO().getKeyAlt();
            double deltaTime = Gdx.graphics.getDeltaTime();

            double amountRotation = deltaTime * (shiftHeld ? 0.2 : 1.0);
            double amountTranslation = deltaTime * (shiftHeld ? 0.05 : 0.4);
            Orientation3DBasics orientationToAdjust = frameBasedGizmoModification.beforeForRotationAdjustment();
            Point3DBasics positionToAdjust = frameBasedGizmoModification.beforeForTranslationAdjustment();

            if (altHeld && !ctrlHeld) // orientation
            {
               if (upArrowHeld) // pitch +
               {
                  orientationToAdjust.appendPitchRotation(amountRotation);
               }
               if (downArrowHeld) // pitch -
               {
                  orientationToAdjust.appendPitchRotation(-amountRotation);
               }
               if (rightArrowHeld) // roll +
               {
                  orientationToAdjust.appendRollRotation(amountRotation);
               }
               if (leftArrowHeld) // roll -
               {
                  orientationToAdjust.appendRollRotation(-amountRotation);
               }
            }
            else if (!altHeld && ctrlHeld) // yaw the orientation, or z the translation
            {
               if (leftArrowHeld) // yaw +
               {
                  orientationToAdjust.appendYawRotation(amountRotation);
               }
               if (rightArrowHeld) // yaw -
               {
                  orientationToAdjust.appendYawRotation(-amountRotation);
               }
               if (upArrowHeld) // z +
               {
                  positionToAdjust.addZ(getTranslateSpeedFactor() * amountTranslation);
               }
               if (downArrowHeld) // z -
               {
                  positionToAdjust.subZ(getTranslateSpeedFactor() * amountTranslation);
               }
            }
            else // translation
            {
               if (upArrowHeld && !ctrlHeld) // x +
               {
                  positionToAdjust.addX(getTranslateSpeedFactor() * amountTranslation);
               }
               if (downArrowHeld && !ctrlHeld) // x -
               {
                  positionToAdjust.subX(getTranslateSpeedFactor() * amountTranslation);
               }
               if (leftArrowHeld && !ctrlHeld) // y +
               {
                  positionToAdjust.addY(getTranslateSpeedFactor() * amountTranslation);
               }
               if (rightArrowHeld && !ctrlHeld) // y -
               {
                  positionToAdjust.subY(getTranslateSpeedFactor() * amountTranslation);
               }
            }

            frameBasedGizmoModification.afterRotationAdjustment(FrameBasedGizmoModification.PREPEND);
            frameBasedGizmoModification.afterRotationAdjustment(FrameBasedGizmoModification.PREPEND);
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

      for (Axis3D axis : Axis3D.values)
      {
         framePose3D.setToZero(gizmoFrame);
         framePose3D.getOrientation().setAndNormalize(axisRotations.get(axis));
         try // Getting an exception here a lot, it's not really a failure, so
         {   // prevent crashing the whole application.
            framePose3D.changeFrame(gizmoFrame.getRootFrame());
         }
         catch (NotARotationMatrixException notARotationMatrixException)
         {
            LogTools.error(notARotationMatrixException.getMessage());
         }
         framePose3D.get(axisTransformToWorlds[axis.ordinal()]);
      }
      // The above Axis calculations actually end up on Z, so we don't have to recalculate this
      framePose3D.get(transformToWorld);
      updateGraphicTransforms();
      LibGDXTools.toEuclid(camera3D.position, cameraPosition);
      distanceToCamera = cameraPosition.distance(framePose3D.getPosition());

      if (resizeAutomatically.get())
      {
         if (!EuclidCoreTools.epsilonEquals(lastDistanceToCamera, distanceToCamera, RDXGizmoTools.ZOOM_RESIZE_EPSILON))
         {
            lastDistanceToCamera = distanceToCamera;
            recreateGraphics();
         }
      }
      if (proportionsNeedUpdate)
      {
         proportionsNeedUpdate = false;
         recreateGraphics();
      }
   }

   private void updateGraphicTransforms()
   {
      LibGDXTools.toLibGDX(transformToWorld, centerSphereModel.getOrCreateModelInstance().transform);
      for (Axis3D axis : Axis3D.values)
      {
         LibGDXTools.toLibGDX(axisTransformToWorlds[axis.ordinal()], arrowModels[axis.ordinal()].getOrCreateModelInstance().transform);
         LibGDXTools.toLibGDX(axisTransformToWorlds[axis.ordinal()], torusModels[axis.ordinal()].getOrCreateModelInstance().transform);
      }
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      closestCollisionSelection = null;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      // Optimization: Do one large sphere collision to avoid completely far off picks
      boundingSphereIntersection.update(1.5 * torusRadius.get(), transformToWorld);
      if (boundingSphereIntersection.intersect(pickRay))
      {
         centerSphereIntersection.update(centerSphereRadius.get(), transformToWorld);
         boolean hoveringCenterSphere = centerSphereIntersection.intersect(pickRay);
         double distance = centerSphereIntersection.getFirstIntersectionToPack().distance(pickRay.getPoint());
         if (hoveringCenterSphere && distance < closestCollisionDistance)
         {
            closestCollisionDistance = distance;
            closestCollisionSelection = SixDoFSelection.CENTER;
            closestCollision.set(centerSphereIntersection.getFirstIntersectionToPack());
         }

         // collide tori
         for (Axis3D axis : Axis3D.values)
         {
            // TODO: Only update when shape changes?
            torusIntersection.update(torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), axisTransformToWorlds[axis.ordinal()]);
            distance = torusIntersection.intersect(pickRay, 100);
            if (!Double.isNaN(distance) && distance < closestCollisionDistance)
            {
               closestCollisionDistance = distance;
               closestCollisionSelection = SixDoFSelection.toAngularSelection(axis);
               closestCollision.set(torusIntersection.getClosestIntersection());
            }
         }

         // collide arrows
         for (Axis3D axis : Axis3D.values)
         {
            for (RobotSide side : RobotSide.values)
            {
               double zOffset = side.negateIfRightSide(0.5 * arrowSpacing + 0.5 * arrowBodyLength);
               // TODO: Only update when shape changes?
               arrowIntersection.update(arrowBodyLength, arrowBodyRadius, arrowHeadRadius, arrowHeadLength, zOffset, axisTransformToWorlds[axis.ordinal()]);
               distance = arrowIntersection.intersect(pickRay, 100, side == RobotSide.LEFT); // only show the cones in the positive direction

               if (!Double.isNaN(distance) && distance < closestCollisionDistance)
               {
                  closestCollisionDistance = distance;
                  closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
                  closestCollision.set(arrowIntersection.getIntersection());
               }
            }
         }
      }
   }

   private void updateMaterialHighlighting()
   {
      // closestCollisionSelection can be null if the the 3D panel is zoomed in or out without the mouse hovering
      boolean highlightingPrior = (isGizmoHovered || isBeingManipulated) && closestCollisionSelection != null;
      // could only do this when selection changed

      if (highlightingPrior && closestCollisionSelection.isCenter())
      {
         centerSphereModel.setMaterial(highlightedMaterials[0]);
      }
      else
      {
         centerSphereModel.setMaterial(normalMaterials[0]);
      }

      for (Axis3D axis : Axis3D.values)
      {
         if (highlightingPrior && closestCollisionSelection.isAngular() && closestCollisionSelection.toAxis3D() == axis)
         {
            torusModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            torusModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }

         if (highlightingPrior && closestCollisionSelection.isLinear() && closestCollisionSelection.toAxis3D() == axis)
         {
            arrowModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            arrowModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }
      }
   }

   public RDXPanel createTunerPanel(String name)
   {
      return new RDXPanel("Pose3D Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
   }

   public void renderImGuiTuner()
   {
      frameBasedGizmoModification.renderImGuiTunerWidgets();

      if (ImGui.collapsingHeader(labels.get("Controls")))
      {
         ImGui.text("Drag using the left mouse button to manipulate the gizmo.");
         // TODO: Add keyboard & scroll wheel controls
      }

      if (ImGui.collapsingHeader(labels.get("Visual options")))
      {
         ImGui.checkbox("Resize based on camera distance", resizeAutomatically);

         ImGui.pushItemWidth(100.00f);
         if (resizeAutomatically.get())
            proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Torus camera size"), torusCameraSize.getData(), 0.001f);
         else
            proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Torus radius"), torusRadius.getData(), 0.001f);
         ImGui.popItemWidth();
         ImGui.separator();
         ImGui.pushItemWidth(100.00f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Center sphere to torus ratio"), centerSphereToTorusRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Torus tube radius ratio"), torusTubeRadiusRatio.getData(), 0.001f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow length ratio"), arrowLengthRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow head body length ratio"), arrowHeadBodyLengthRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow head body radius ratio"), arrowHeadBodyRadiusRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow spacing factor"), arrowSpacingFactor.getData(), 0.05f);
         ImGui.popItemWidth();
      }
   }

   private void recreateGraphics()
   {
      if (resizeAutomatically.get())
      {
         if (lastDistanceToCamera > 0.0)
            torusRadius.set(torusCameraSize.get() * (float) lastDistanceToCamera);
         else
            torusRadius.set(torusCameraSize.get());
      }
      centerSphereRadius.set(torusRadius.get() * centerSphereToTorusRatio.get());
      arrowBodyRadius = torusTubeRadiusRatio.get() * torusRadius.get();
      arrowLength = arrowLengthRatio.get() * torusRadius.get();
      arrowBodyLength = (1.0 - arrowHeadBodyLengthRatio.get()) * arrowLength;
      arrowHeadRadius = arrowHeadBodyRadiusRatio.get() * arrowBodyRadius;
      arrowHeadLength = arrowHeadBodyLengthRatio.get() * arrowLength;
      arrowSpacing = arrowSpacingFactor.get() * (torusRadius.get() + (torusTubeRadiusRatio.get() * torusRadius.get()));

      updateMaterialHighlighting();

      centerSphereModel.invalidateMesh();
      for (Axis3D axis : Axis3D.values)
      {
         arrowModels[axis.ordinal()].invalidateMesh();
         torusModels[axis.ordinal()].invalidateMesh();
      }
      updateGraphicTransforms();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      centerSphereModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      for (Axis3D axis : Axis3D.values)
      {
         arrowModels[axis.ordinal()].getOrCreateModelInstance().getRenderables(renderables, pool);
         torusModels[axis.ordinal()].getOrCreateModelInstance().getRenderables(renderables, pool);
      }
   }

   public FramePose3DReadOnly getPose()
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

   public static Mesh angularHighlightMesh(double majorRadius, double minorRadius)
   {
      return tetrahedronRingMesh(1.75 * minorRadius, 1.25 * minorRadius, 5);
   }

   public static Mesh linearControlHighlightMesh(double bodyRadius, double bodyLength, double spacing)
   {
      RDXMeshBuilder meshBuilder = new RDXMeshBuilder();

      int numberOfHighlights = 5;

      Point3D center = new Point3D(0, 0, 0.5 * spacing + 0.33 * bodyLength);
      MeshDataHolder ringMesh = tetrahedronRingMeshDataHolder(1.75 * bodyRadius, 1.25 * bodyRadius, numberOfHighlights);
      meshBuilder.addMesh(ringMesh, center);
      center.negate();
      meshBuilder.addMesh(ringMesh, center);

      return meshBuilder.generateMesh();
   }

   public static Mesh tetrahedronRingMesh(double ringRadius, double tetrahedronSize, int numberOfTetrahedrons)
   {
      return RDXMeshDataInterpreter.interpretMeshData(tetrahedronRingMeshDataHolder(ringRadius, tetrahedronSize, numberOfTetrahedrons));
   }

   public static MeshDataHolder tetrahedronRingMeshDataHolder(double ringRadius, double tetrahedronSize, int numberOfTetrahedrons)
   {
      RDXMeshBuilder meshBuilder = new RDXMeshBuilder();

      Point3D position = new Point3D();
      Point3D offset = new Point3D();
      Quaternion orientation = new Quaternion();

      for (int i = 0; i < numberOfTetrahedrons; i++)
      {
         MeshDataHolder tetrahedron = MeshDataGenerator.Tetrahedron(tetrahedronSize);
         orientation.setToYawOrientation(i * 2.0 * Math.PI / numberOfTetrahedrons);
         orientation.appendPitchRotation(0.5 * Math.PI);

         offset.set(0.0, 0.0, ringRadius);
         orientation.transform(offset);
         position.set(offset);
         meshBuilder.addMesh(tetrahedron, position, orientation);
      }

      return meshBuilder.generateMeshDataHolder();
   }

   private double getTranslateSpeedFactor()
   {
      return translateSpeedFactor * distanceToCamera;
   }

   public void setResizeAutomatically(boolean resizeAutomatically)
   {
      this.resizeAutomatically.set(resizeAutomatically);
   }

   ClockFaceRotation3DMouseDragAlgorithm getClockFaceDragAlgorithm()
   {
      return clockFaceDragAlgorithm;
   }

   public Notification getGizmoModifiedByUser()
   {
      return gizmoModifiedByUser;
   }
}
