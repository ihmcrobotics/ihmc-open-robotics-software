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
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.tools.UnitConversions;

import java.util.Random;

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
   private int closestCollisionSelection = -1;
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
   /** For being able to tell if the user moved the gizmo. */
   private final Notification gizmoModifiedByUser = new Notification();
   private RDXFocusBasedCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private final Plane3DMouseDragAlgorithm planeDragAlgorithm = new Plane3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private boolean hollowCylinderIntersects;
   private boolean positiveXArrowIntersects;
   private boolean positiveYArrowIntersects;
   private boolean negativeXArrowIntersects;
   private boolean negativeYArrowIntersects;
   private boolean showArrows = true;
   private boolean highlightingEnabled = true;
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
      camera3D = panel3D.getCamera3D();
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
      material.set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      material.set(new BlendingAttribute(true, alpha));
      return material;
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
            determineCurrentSelectionFromPickRay(pickRay);
         }

         if (closestCollisionSelection > -1)
         {
            pickResult.setDistanceToCamera(closestCollisionDistance);
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean isWindowHovered = input.isWindowHovered();
      int yawMouseButton = ImGuiMouseButton.Right;
      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(yawMouseButton);

      isGizmoHovered = isWindowHovered && pickResult == input.getClosestPick();

      if (isGizmoHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
      {
         queuePopupToOpen = true;
      }

      boolean isRingHovered = isGizmoHovered && closestCollisionSelection == 0;
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

      isBeingManipulated = (translateDragData.getObjectBeingDragged() == this) || (yawDragData.getObjectBeingDragged() == this);
      if (isBeingManipulated)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (translateDragData.isDragging())
         {
            Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
            adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            adjustmentPose3D.getPosition().add(planarMotion);
            adjustmentNeedsToBeApplied = true;
            closestCollision.add(planarMotion);
         }
         else // yaw dragging
         {
            if (clockFaceDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z, transformToWorld))
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
         if (anyArrowHeld)
         {
            boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
            boolean altHeld = ImGui.getIO().getKeyAlt();
            boolean shiftHeld = ImGui.getIO().getKeyShift();
            double deltaTime = Gdx.graphics.getDeltaTime();

            if (altHeld) // orientation
            {
               double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
               Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
               if (leftArrowHeld) // yaw +
               {
                  orientationToAdjust.appendYawRotation(amount);
               }
               if (rightArrowHeld) // yaw -
               {
                  orientationToAdjust.appendYawRotation(-amount);
               }
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

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      hollowCylinderIntersects = false;
      positiveXArrowIntersects = false;
      positiveYArrowIntersects = false;
      negativeXArrowIntersects = false;
      negativeYArrowIntersects = false;
      closestCollisionSelection = -1;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      hollowCylinderIntersection.update(discThickness.get(), discOuterRadius.get(), discInnerRadius.get(), discThickness.get() / 2.0, transformToWorld);
      double distance = hollowCylinderIntersection.intersect(pickRay);
      if (!Double.isNaN(distance) && distance < closestCollisionDistance)
      {
         hollowCylinderIntersects = true;
         closestCollisionDistance = distance;
         closestCollisionSelection = 0;
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
            positiveXArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 1;
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
            positiveYArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 2;
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
            negativeXArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 3;
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
            negativeYArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 4;
            closestCollision.set(negativeYArrowIntersection.getFirstIntersectionToPack());
         }
      }

      updateMaterialHighlighting();
   }

   private void updateMaterialHighlighting()
   {
      boolean prior = highlightingEnabled && isGizmoHovered;
      discModel.setMaterial(prior && closestCollisionSelection == 0 ? highlightedMaterial : normalMaterial);
      positiveXArrowModel.setMaterial(prior && closestCollisionSelection == 1 ? highlightedMaterial : normalMaterial);
      positiveYArrowModel.setMaterial(prior && closestCollisionSelection == 2 ? highlightedMaterial : normalMaterial);
      negativeXArrowModel.setMaterial(prior && closestCollisionSelection == 3 ? highlightedMaterial : normalMaterial);
      negativeYArrowModel.setMaterial(prior && closestCollisionSelection == 4 ? highlightedMaterial : normalMaterial);
   }

   public ImGuiPanel createTunerPanel(String name)
   {
      return new ImGuiPanel("Footstep Ring Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
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
      yawImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      boolean inputChanged = yawImGuiInput.getInputChanged();
      adjustmentNeedsToBeApplied |= inputChanged;
      if (inputChanged)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getDoubleValue()), 0.0, 0.0);
         afterRotationAdjustment(SET_ABSOLUTE);
      }
      boolean rotationStepped = yawImGuiInput.getStepButtonClicked();
      adjustmentNeedsToBeApplied |= rotationStepped;
      if (rotationStepped)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getSteppedAmount()), 0.0, 0.0);
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
      return isGizmoHovered
             && (hollowCylinderIntersects || positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects);
   }

   public boolean getAnyArrowPickSelected()
   {
      return isGizmoHovered && (positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects);
   }

   public boolean getHollowCylinderHovered()
   {
      return isGizmoHovered && hollowCylinderIntersects;
   }

   public boolean getPositiveXArrowHovered()
   {
      return isGizmoHovered && positiveXArrowIntersects;
   }

   public boolean getPositiveYArrowHovered()
   {
      return isGizmoHovered && positiveYArrowIntersects;
   }

   public boolean getNegativeXArrowHovered()
   {
      return isGizmoHovered && negativeXArrowIntersects;
   }

   public boolean getNegativeYArrowHovered()
   {
      return isGizmoHovered && negativeYArrowIntersects;
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

   public boolean getGizmoHovered()
   {
      return isGizmoHovered;
   }

   public Notification getGizmoModifiedByUser()
   {
      return gizmoModifiedByUser;
   }
}
