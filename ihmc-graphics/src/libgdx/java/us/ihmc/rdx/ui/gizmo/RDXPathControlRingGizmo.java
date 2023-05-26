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
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

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
   /**
    * The main, source, true, base transform that this thing represents.
    */
   private RigidBodyTransform transformToParent;
   private ReferenceFrame parentReferenceFrame;
   private ReferenceFrame gizmoFrame;
   private final RigidBodyTransform controlRingTransformToWorld = new RigidBodyTransform();
   private final FramePose3D controlRingPose = new FramePose3D();
   private final FramePose3D tempFramePose3D = new FramePose3D();
   private RDXFocusBasedCamera camera3D;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromKeyboardTransformationToWorld = new RigidBodyTransform();
   private ModifiableReferenceFrame keyboardTransformationFrame;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private double lastDistanceToCamera = -1.0;
   private final Plane3DMouseDragAlgorithm planeDragAlgorithm = new Plane3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private boolean hollowCylinderIntersects;
   private boolean positiveXArrowIntersects;
   private boolean positiveYArrowIntersects;
   private boolean negativeXArrowIntersects;
   private boolean negativeYArrowIntersects;
   private boolean showArrows = true;
   private boolean highlightingEnabled = true;
   private boolean isNewlyModified;
   private final double translateSpeedFactor = 0.5;
   private boolean queuePopupToOpen = false;

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
      keyboardTransformationFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public void setParentFrame(ReferenceFrame parentReferenceFrame)
   {
      this.parentReferenceFrame = parentReferenceFrame;
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
   }

   public void create(RDX3DPanel panel3D)
   {
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
      updateTransforms();

      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(ImGuiMouseButton.Right);

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

   public void process3DViewInput(ImGui3DViewInput input)
   {
      process3DViewInput(input, true);
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean allowUserInput)
   {
      updateTransforms();

      boolean isWindowHovered = input.isWindowHovered();
      int yawMouseButton = ImGuiMouseButton.Right;
      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(yawMouseButton);

      isNewlyModified = false;
      isGizmoHovered = input.isWindowHovered() && pickResult == input.getClosestPick();
      boolean isRingHovered = isGizmoHovered && closestCollisionSelection == 0;
      boolean leftButtonDown = ImGui.isMouseDown(ImGuiMouseButton.Left);
      boolean rightButtonDown = ImGui.isMouseDown(yawMouseButton);

      if (allowUserInput)
      {
         if (isGizmoHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
         {
            queuePopupToOpen = true;
         }

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

         isBeingManipulated = (yawDragData.getObjectBeingDragged() == this && rightButtonDown)
                           || (translateDragData.getObjectBeingDragged() == this && leftButtonDown);
         if (isBeingManipulated)
         {
            isNewlyModified = true;
            Line3DReadOnly pickRay = input.getPickRayInWorld();

            if (translateDragData.isDragging())
            {
               Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
               tempFramePose3D.setToZero(gizmoFrame);
               tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
               tempFramePose3D.getPosition().add(planarMotion);
               tempFramePose3D.changeFrame(parentReferenceFrame);
               tempFramePose3D.get(transformToParent);
               closestCollision.add(planarMotion);
            }
            else // yaw dragging
            {
               if (clockFaceDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z, controlRingPose))
               {
                  tempFramePose3D.setToZero(gizmoFrame);
                  tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
                  clockFaceDragAlgorithm.getMotion().transform(tempFramePose3D.getOrientation());
                  tempFramePose3D.changeFrame(parentReferenceFrame);
                  tempFramePose3D.get(transformToParent);
               }
            }
         }

         if (isWindowHovered)
         {
            // keyboard based controls
            boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
            boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
            boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
            boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
            boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
            isNewlyModified |= anyArrowHeld;
            boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
            boolean altHeld = ImGui.getIO().getKeyAlt();
            boolean shiftHeld = ImGui.getIO().getKeyShift();
            double deltaTime = Gdx.graphics.getDeltaTime();

            if (altHeld) // orientation
            {
               double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
               if (leftArrowHeld) // yaw +
               {
                  transformToParent.getRotation().appendYawRotation(amount);
               }
               if (rightArrowHeld) // yaw -
               {
                  transformToParent.getRotation().appendYawRotation(-amount);
               }
            }
            else if (anyArrowHeld) // translation (only the arrow keys do the moving)
            {
               transformFromKeyboardTransformationToWorld.setToZero();
               transformFromKeyboardTransformationToWorld.getRotation().setToYawOrientation(camera3D.getFocusPointPose().getYaw());
               keyboardTransformationFrame.getReferenceFrame().update();
               tempFramePose3D.setToZero(keyboardTransformationFrame.getReferenceFrame());

               double amount = deltaTime * (shiftHeld ? 0.05 : 0.4);
               if (upArrowHeld && !ctrlHeld) // x +
               {
                  tempFramePose3D.getPosition().addX(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && !ctrlHeld) // x -
               {
                  tempFramePose3D.getPosition().subX(getTranslateSpeedFactor() * amount);
               }
               if (leftArrowHeld) // y +
               {
                  tempFramePose3D.getPosition().addY(getTranslateSpeedFactor() * amount);
               }
               if (rightArrowHeld) // y -
               {
                  tempFramePose3D.getPosition().subY(getTranslateSpeedFactor() * amount);
               }
               if (upArrowHeld && ctrlHeld) // z +
               {
                  tempFramePose3D.getPosition().addZ(getTranslateSpeedFactor() * amount);
               }
               if (downArrowHeld && ctrlHeld) // z -
               {
                  tempFramePose3D.getPosition().subZ(getTranslateSpeedFactor() * amount);
               }

               tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
               tempFramePose3D.get(tempTransform);
               transformToParent.getTranslation().add(tempTransform.getTranslation());
            }
         }
      }

      // after things have been modified, update the derivative stuff
      updateTransforms();

      LibGDXTools.toEuclid(camera3D.position, cameraPosition);
      distanceToCamera = cameraPosition.distance(controlRingPose.getPosition());
      if (lastDistanceToCamera != distanceToCamera)
      {
         lastDistanceToCamera = distanceToCamera;
         recreateGraphics();
         updateTransforms();
      }
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

   public void updateTransforms()
   {
      gizmoFrame.update();
      // keeping the gizmo on the X-Y plane
      tempFramePose3D.setToZero(gizmoFrame);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose3D.getOrientation().setToYawOrientation(tempFramePose3D.getOrientation().getYaw());
      tempFramePose3D.changeFrame(parentReferenceFrame);
      tempFramePose3D.get(transformToParent);
      gizmoFrame.update();
      controlRingPose.setToZero(gizmoFrame);
      controlRingPose.changeFrame(ReferenceFrame.getWorldFrame());
      controlRingPose.get(controlRingTransformToWorld);
      LibGDXTools.toLibGDX(controlRingTransformToWorld, discModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(controlRingTransformToWorld, positiveXArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(controlRingTransformToWorld, positiveYArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(controlRingTransformToWorld, negativeXArrowModel.getOrCreateModelInstance().transform);
      LibGDXTools.toLibGDX(controlRingTransformToWorld, negativeYArrowModel.getOrCreateModelInstance().transform);
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

      hollowCylinderIntersection.update(discThickness.get(), discOuterRadius.get(), discInnerRadius.get(), discThickness.get() / 2.0,
                                        controlRingTransformToWorld);
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
                                           new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN), controlRingTransformToWorld);
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
                                           new YawPitchRoll(0.0, 0.0, -QUARTER_TURN), controlRingTransformToWorld);
         distance = positiveYArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            positiveYArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 2;
            closestCollision.set(positiveYArrowIntersection.getClosestIntersection());
         }
         temporaryTailTransform.set(xArrowTailTransform);
         controlRingTransformToWorld.transform(temporaryTailTransform);
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
         controlRingTransformToWorld.transform(temporaryTailTransform);
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
      ImGui.text("Use the right mouse button to manipulate the widget.");

      if (ImGui.button(labels.get("Reset")))
      {
         transformToParent.setToZero();
      }

      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      proportionsChanged |= ImGui.dragFloat(labels.get("Disc outer radius"), discOuterRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Disc inner radius"), discInnerRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Disc thickness"), discThickness.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Arrow width"), arrowWidth.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Arrow height"), arrowHeight.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Arrow spacing"), arrowSpacing.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Arrow tail width ratio"), arrowTailWidthRatio.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(labels.get("Arrow tail length ratio"), arrowTailLengthRatio.getData(), 0.001f, 0.0f, 1000.0f);
      ImGui.popItemWidth();

      if (proportionsChanged)
         recreateGraphics();

      updateTransforms();
   }

   private void recreateGraphics()
   {
      updateMaterialHighlighting();
      discModel.invalidateMesh();
      positiveXArrowModel.invalidateMesh();
      positiveYArrowModel.invalidateMesh();
      negativeXArrowModel.invalidateMesh();
      negativeYArrowModel.invalidateMesh();
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
      return controlRingPose;
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

   public boolean getAnyPartPickSelected()
   {
      return isGizmoHovered
             && (hollowCylinderIntersects || positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects);
   }

   public boolean getAnyArrowPickSelected()
   {
      return isGizmoHovered && (positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects);
   }

   public boolean getHollowCylinderPickSelected()
   {
      return isGizmoHovered && hollowCylinderIntersects;
   }

   public boolean getPositiveXArrowPickSelected()
   {
      return isGizmoHovered && positiveXArrowIntersects;
   }

   public boolean getPositiveYArrowPickSelected()
   {
      return isGizmoHovered && positiveYArrowIntersects;
   }

   public boolean getNegativeXArrowPickSelected()
   {
      return isGizmoHovered && negativeXArrowIntersects;
   }

   public boolean getNegativeYArrowPickSelected()
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

   public boolean isNewlyModified()
   {
      return isNewlyModified;
   }

   public boolean getGizmoHovered()
   {
      return isGizmoHovered;
   }
}
