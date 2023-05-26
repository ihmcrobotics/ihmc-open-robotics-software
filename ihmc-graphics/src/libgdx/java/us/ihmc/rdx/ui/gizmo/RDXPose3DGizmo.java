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
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMeshBuilder;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;

import java.util.Random;

import static us.ihmc.rdx.ui.gizmo.RDXGizmoTools.AXIS_COLORS;
import static us.ihmc.rdx.ui.gizmo.RDXGizmoTools.AXIS_SELECTED_COLORS;

public class RDXPose3DGizmo implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat torusRadius = new ImFloat(0.5f);
   private final ImFloat torusCameraSize = new ImFloat(0.067f);
   private final ImFloat torusTubeRadiusRatio = new ImFloat(0.074f);
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
   private final DynamicLibGDXModel[] arrowModels = new DynamicLibGDXModel[3];
   private final DynamicLibGDXModel[] torusModels = new DynamicLibGDXModel[3];
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection;
   private double closestCollisionDistance;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean isGizmoHovered = false;
   private boolean isBeingManipulated = false;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private final DiscreteArrowRayIntersection arrowIntersection = new DiscreteArrowRayIntersection();
   /** The main, source, true, base transform that this thing represents. */
   private RigidBodyTransform transformToParent;
   /** This pose 3D should always be left in world frame and represent this gizmo's pose. */
   private final FramePose3D framePose3D = new FramePose3D();
   /** Pose used for making adjustments. */
   private final FramePose3D adjustmentPose3D = new FramePose3D();
   /** Used for adjusting orientation with respect to different frames. */
   private final FrameQuaternion rotationAdjustmentQuaternion = new FrameQuaternion();
   /** Used to move the gizmo with respect to where you are looking at the scene from. */
   private final ModifiableReferenceFrame cameraZUpFrameForAdjustment = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private ReferenceFrame parentReferenceFrame;
   private ReferenceFrame gizmoFrame;
   /** Gizmo transform to world so it can be calculated once. */
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   /** Transforms to world for placing the graphics. */
   private final RigidBodyTransform[] axisTransformToWorlds = new RigidBodyTransform[3];
   /** For being able to tell if the user moved the gizmo. */
   private final Notification gizmoModifiedByUser = new Notification();
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private final Line3DMouseDragAlgorithm lineDragAlgorithm = new Line3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private RDXFocusBasedCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private double lastDistanceToCamera = -1.0;
   private final double translateSpeedFactor = 0.5;
   private boolean queuePopupToOpen = false;
   private final Random random = new Random();
   private boolean proportionsNeedUpdate = false;
   private boolean adjustmentNeedsToBeApplied = false;
   private static final boolean SET_ABSOLUTE = false;
   private static final boolean PREPEND = true;
   private RDXPose3DGizmoAdjustmentFrame translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
   private RDXPose3DGizmoAdjustmentFrame rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
   private ImGuiInputDouble translationStepSizeInput;
   private ImGuiInputDouble positionXImGuiInput;
   private ImGuiInputDouble positionYImGuiInput;
   private ImGuiInputDouble positionZImGuiInput;
   private ImGuiInputDouble rotationStepSizeInput;
   private ImGuiInputDoubleForRotations yawImGuiInput;
   private ImGuiInputDoubleForRotations pitchImGuiInput;
   private ImGuiInputDoubleForRotations rollImGuiInput;

   public RDXPose3DGizmo()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public RDXPose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      ReferenceFrame gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
      initialize(gizmoFrame, transformToParent);
   }

   public RDXPose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   public RDXPose3DGizmo(RigidBodyTransform gizmoTransformToParentFrameToModify, ReferenceFrame parentReferenceFrame)
   {
      ReferenceFrame gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame,
                                                                                                         gizmoTransformToParentFrameToModify);
      initialize(gizmoFrame, gizmoTransformToParentFrameToModify);
   }

   private void initialize(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this.parentReferenceFrame = gizmoFrame.getParent();
      this.transformToParent = gizmoTransformToParentFrameToModify;
      this.gizmoFrame = gizmoFrame;
   }

   public void setParentFrame(ReferenceFrame parentReferenceFrame)
   {
      this.parentReferenceFrame = parentReferenceFrame;
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
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
      translationStepSizeInput = new ImGuiInputDouble("Translation step size", "%.5f", 0.01);
      positionXImGuiInput = new ImGuiInputDouble("X", "%.5f");
      positionYImGuiInput = new ImGuiInputDouble("Y", "%.5f");
      positionZImGuiInput = new ImGuiInputDouble("Z", "%.5f");
      rotationStepSizeInput = new ImGuiInputDouble("Rotation step size " + UnitConversions.DEGREE_SYMBOL, "%.5f", 0.5);
      yawImGuiInput = new ImGuiInputDoubleForRotations("Yaw " + UnitConversions.DEGREE_SYMBOL, "%.5f");
      pitchImGuiInput = new ImGuiInputDoubleForRotations("Pitch " + UnitConversions.DEGREE_SYMBOL, "%.5f");
      rollImGuiInput = new ImGuiInputDoubleForRotations("Roll " + UnitConversions.DEGREE_SYMBOL, "%.5f");
      camera3D = panel3D.getCamera3D();
      panel3D.addImGuiOverlayAddition(this::renderTooltipAndContextMenu);

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

      updateMaterialHighlighting();

      if (isGizmoHovered && manipulationDragData.getDragJustStarted())
      {
         clockFaceDragAlgorithm.reset();
         manipulationDragData.setObjectBeingDragged(this);
      }

      isBeingManipulated = manipulationDragData.getObjectBeingDragged() == this;
      if (isBeingManipulated)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (closestCollisionSelection.isLinear())
         {
            Vector3DReadOnly linearMotion = lineDragAlgorithm.calculate(pickRay,
                                                                        closestCollision,
                                                                        axisRotations.get(closestCollisionSelection.toAxis3D()),
                                                                        transformToWorld);

            adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            adjustmentPose3D.getPosition().add(linearMotion);
            adjustmentNeedsToBeApplied = true;
            closestCollision.add(linearMotion);
         }
         else if (closestCollisionSelection.isAngular())
         {
            if (clockFaceDragAlgorithm.calculate(pickRay,
                                                 closestCollision,
                                                 axisRotations.get(closestCollisionSelection.toAxis3D()),
                                                 transformToWorld))
            {
               adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
               clockFaceDragAlgorithm.getMotion().transform(adjustmentPose3D.getOrientation());
               adjustmentNeedsToBeApplied = true;
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
            Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
            orientationToAdjust.appendYawRotation(Math.signum(deltaScroll) * speed * Math.PI);
            afterRotationAdjustment(PREPEND);
            adjustmentNeedsToBeApplied = true;
         }

         // keyboard based controls
         boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
         boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
         boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
         boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
         boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
         if (anyArrowHeld) // only the arrow keys do the moving
         {
            boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
            boolean altHeld = ImGui.getIO().getKeyAlt();
            boolean shiftHeld = ImGui.getIO().getKeyShift();
            double deltaTime = Gdx.graphics.getDeltaTime();
            if (altHeld) // orientation
            {
               double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
               Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
               if (upArrowHeld) // pitch +
               {
                  orientationToAdjust.appendPitchRotation(amount);
               }
               if (downArrowHeld) // pitch -
               {
                  orientationToAdjust.appendPitchRotation(-amount);
               }
               if (rightArrowHeld && !ctrlHeld) // roll +
               {
                  orientationToAdjust.appendRollRotation(amount);
               }
               if (leftArrowHeld && !ctrlHeld) // roll -
               {
                  orientationToAdjust.appendRollRotation(-amount);
               }
               if (leftArrowHeld && ctrlHeld) // yaw +
               {
                  orientationToAdjust.appendYawRotation(amount);
               }
               if (rightArrowHeld && ctrlHeld) // yaw -
               {
                  orientationToAdjust.appendYawRotation(-amount);
               }
               afterRotationAdjustment(PREPEND);
            }
            else // translation
            {
               double amount = deltaTime * (shiftHeld ? 0.05 : 0.4);
               beforeForTranslationAdjustment();
               if (upArrowHeld && !ctrlHeld) // x +
               {
                  adjustmentPose3D.getPosition().addX(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && !ctrlHeld) // x -
               {
                  adjustmentPose3D.getPosition().subX(getTranslateSpeedFactor() * amount);
               }
               if (leftArrowHeld) // y +
               {
                  adjustmentPose3D.getPosition().addY(getTranslateSpeedFactor() * amount);
               }
               if (rightArrowHeld) // y -
               {
                  adjustmentPose3D.getPosition().subY(getTranslateSpeedFactor() * amount);
               }
               if (upArrowHeld && ctrlHeld) // z +
               {
                  adjustmentPose3D.getPosition().addZ(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && ctrlHeld) // z -
               {
                  adjustmentPose3D.getPosition().subZ(getTranslateSpeedFactor() * amount);
               }
            }

            adjustmentNeedsToBeApplied = true;
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
      if (adjustmentNeedsToBeApplied)
      {
         adjustmentNeedsToBeApplied = false;
         adjustmentPose3D.changeFrame(parentReferenceFrame);
         adjustmentPose3D.get(transformToParent);
         gizmoModifiedByUser.set();
      }

      gizmoFrame.update();
      adjustmentPose3D.setToZero(gizmoFrame);

      for (Axis3D axis : Axis3D.values)
      {
         framePose3D.setToZero(gizmoFrame);
         framePose3D.getOrientation().setAndNormalize(axisRotations.get(axis));
         framePose3D.changeFrame(ReferenceFrame.getWorldFrame());
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
         // collide tori
         for (Axis3D axis : Axis3D.values)
         {
            // TODO: Only update when shape changes?
            torusIntersection.update(torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), axisTransformToWorlds[axis.ordinal()]);
            double distance = torusIntersection.intersect(pickRay, 100);
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
               double distance = arrowIntersection.intersect(pickRay, 100, side == RobotSide.LEFT); // only show the cones in the positive direction

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
      boolean prior = (isGizmoHovered || isBeingManipulated) && closestCollisionSelection != null;
      // could only do this when selection changed
      for (Axis3D axis : Axis3D.values)
      {
         if (prior && closestCollisionSelection.isAngular() && closestCollisionSelection.toAxis3D() == axis)
         {
            torusModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            torusModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }

         if (prior && closestCollisionSelection.isLinear() && closestCollisionSelection.toAxis3D() == axis)
         {
            arrowModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            arrowModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }
      }
   }

   public ImGuiPanel createTunerPanel(String name)
   {
      return new ImGuiPanel("Pose3D Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
   }

   public void renderImGuiTuner()
   {
      ImGui.text("Parent frame: " + parentReferenceFrame.getName());

      ImGui.text("Translation adjustment frame:");
      if (ImGui.radioButton(labels.get("Camera Z Up", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Camera", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.WORLD;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Parent", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.PARENT;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Local", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.LOCAL))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.LOCAL;
      ImGui.text("Rotation adjustment frame:");
      if (ImGui.radioButton(labels.get("Camera Z Up", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Camera", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.WORLD;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Parent", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.PARENT;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Local", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.LOCAL))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.LOCAL;

      beforeForTranslationAdjustment();
      translationStepSizeInput.render(RDXGizmoTools.INITIAL_SMALL_STEP, RDXGizmoTools.INITIAL_BIG_STEP);
      positionXImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getX());
      positionYImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getY());
      positionZImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getZ());
      adjustmentNeedsToBeApplied |= positionXImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentNeedsToBeApplied |= positionYImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentNeedsToBeApplied |= positionZImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentPose3D.getPosition().set(positionXImGuiInput.getDoubleValue(), positionYImGuiInput.getDoubleValue(), positionZImGuiInput.getDoubleValue());

      rotationStepSizeInput.render(RDXGizmoTools.INITIAL_FINE_ROTATION, RDXGizmoTools.INITIAL_COARSE_ROTATION);
      Orientation3DBasics orientationToPrint = getOrientationInAdjustmentFrame();
      yawImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getYaw()));
      pitchImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getPitch()));
      rollImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getRoll()));
      yawImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      pitchImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      rollImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      boolean inputChanged = yawImGuiInput.getInputChanged();
      inputChanged |= pitchImGuiInput.getInputChanged();
      inputChanged |= rollImGuiInput.getInputChanged();
      adjustmentNeedsToBeApplied |= inputChanged;
      if (inputChanged)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getDoubleValue()),
                                             Math.toRadians(pitchImGuiInput.getDoubleValue()),
                                             Math.toRadians(rollImGuiInput.getDoubleValue()));
         afterRotationAdjustment(SET_ABSOLUTE);
      }
      boolean rotationStepped = yawImGuiInput.getStepButtonClicked();
      rotationStepped |= pitchImGuiInput.getStepButtonClicked();
      rotationStepped |= rollImGuiInput.getStepButtonClicked();
      adjustmentNeedsToBeApplied |= rotationStepped;
      if (rotationStepped)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getSteppedAmount()),
                                             Math.toRadians(pitchImGuiInput.getSteppedAmount()),
                                             Math.toRadians(rollImGuiInput.getSteppedAmount()));
         afterRotationAdjustment(PREPEND);
      }

      ImGui.text("Set to zero in:");
      ImGui.sameLine();
      if (ImGui.button("World"))
      {
         adjustmentPose3D.setToZero(ReferenceFrame.getWorldFrame());
         adjustmentNeedsToBeApplied = true;
      }
      ImGui.sameLine();
      if (ImGui.button("Parent"))
      {
         adjustmentPose3D.setToZero(parentReferenceFrame);
         adjustmentNeedsToBeApplied = true;
      }

      if (ImGui.collapsingHeader(labels.get("Controls")))
      {
         ImGui.text("Drag using the left mouse button to manipulate the gizmo.");
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
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Torus tube radius ratio"), torusTubeRadiusRatio.getData(), 0.001f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow length ratio"), arrowLengthRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow head body length ratio"), arrowHeadBodyLengthRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow head body radius ratio"), arrowHeadBodyRadiusRatio.getData(), 0.05f);
         proportionsNeedUpdate |= ImGui.dragFloat(labels.get("Arrow spacing factor"), arrowSpacingFactor.getData(), 0.05f);
         ImGui.popItemWidth();
      }
   }

   private void beforeForTranslationAdjustment()
   {
      switch (translationAdjustmentFrame)
      {
         case WORLD -> adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
         case PARENT -> adjustmentPose3D.changeFrame(parentReferenceFrame);
         case CAMERA_ZUP ->
         {
            prepareCameraZUpFrameForAdjustment();
            adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
         }
         case CAMERA ->
         {
            adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
         }
      }
   }

   /**
    * Used to show the user the current rotation with respect to the current adjustment frame.
    */
   private Orientation3DBasics getOrientationInAdjustmentFrame()
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               adjustmentPose3D.changeFrame(parentReferenceFrame);
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               prepareCameraZUpFrameForAdjustment();
               adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
            }
         }
         default -> // LOCAL
         {
            adjustmentPose3D.changeFrame(gizmoFrame);
         }
      }
      return adjustmentPose3D.getOrientation();
   }

   /**
    * Call before performing an adjustment of this pose's orientation.
    * @return orientation to adjust
    */
   private Orientation3DBasics beforeForRotationAdjustment()
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               rotationAdjustmentQuaternion.setToZero(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               rotationAdjustmentQuaternion.setToZero(parentReferenceFrame);
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               prepareCameraZUpFrameForAdjustment();
               rotationAdjustmentQuaternion.setToZero(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               rotationAdjustmentQuaternion.setToZero(camera3D.getCameraFrame());
            }

            return rotationAdjustmentQuaternion;
         }
         default -> // LOCAL
         {
            adjustmentPose3D.changeFrame(gizmoFrame);
            return adjustmentPose3D.getOrientation();
         }
      }
   }

   private void prepareCameraZUpFrameForAdjustment()
   {
      cameraZUpFrameForAdjustment.getTransformToParent().getTranslation().set(camera3D.getCameraPose().getPosition());
      cameraZUpFrameForAdjustment.getTransformToParent().getRotation().setToYawOrientation(camera3D.getCameraPose().getYaw());
      cameraZUpFrameForAdjustment.getReferenceFrame().update();
   }

   /**
    * Call after performing an adjustment on the orientation returned by {@link #beforeForRotationAdjustment}
    * @param prepend true if the adjustment is meant to be a nudge, false if the adjustment is meant to fully replace
    *                the current orientation. Use the constants {@link #SET_ABSOLUTE} and {@link #PREPEND}.
    */
   private void afterRotationAdjustment(boolean prepend)
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               rotationAdjustmentQuaternion.changeFrame(ReferenceFrame.getWorldFrame());
               adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               rotationAdjustmentQuaternion.changeFrame(parentReferenceFrame);
               adjustmentPose3D.changeFrame(parentReferenceFrame);
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               rotationAdjustmentQuaternion.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
               adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               rotationAdjustmentQuaternion.changeFrame(camera3D.getCameraFrame());
               adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
            }
            if (prepend == PREPEND)
               adjustmentPose3D.getOrientation().prepend(rotationAdjustmentQuaternion);
            else // SET_ABSOLUTE
               adjustmentPose3D.getOrientation().set(rotationAdjustmentQuaternion);
         }
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
      arrowBodyRadius = (float) torusTubeRadiusRatio.get() * torusRadius.get();
      arrowLength = arrowLengthRatio.get() * torusRadius.get();
      arrowBodyLength = (1.0 - arrowHeadBodyLengthRatio.get()) * arrowLength;
      arrowHeadRadius = arrowHeadBodyRadiusRatio.get() * arrowBodyRadius;
      arrowHeadLength = arrowHeadBodyLengthRatio.get() * arrowLength;
      arrowSpacing = arrowSpacingFactor.get() * (torusRadius.get() + (torusTubeRadiusRatio.get() * torusRadius.get()));

      updateMaterialHighlighting();

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
