package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXPathControlRingGizmo implements RenderableProvider
{
   public static final Color DISC_COLOR = RDXGizmoTools.CENTER_DEFAULT_COLOR;
   public static final Color X_ARROW_COLOR = RDXGizmoTools.X_AXIS_DEFAULT_COLOR;
   public static final Color Y_ARROW_COLOR = RDXGizmoTools.Y_AXIS_DEFAULT_COLOR;

   private final double QUARTER_TURN = Math.PI / 2.0;
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
   private DynamicLibGDXModel discModel = new DynamicLibGDXModel();
   private DynamicLibGDXModel positiveXArrowModel = new DynamicLibGDXModel();
   private DynamicLibGDXModel positiveYArrowModel = new DynamicLibGDXModel();
   private DynamicLibGDXModel negativeXArrowModel = new DynamicLibGDXModel();
   private DynamicLibGDXModel negativeYArrowModel = new DynamicLibGDXModel();

   // Transforms of arrows
   private final RigidBodyTransform xArrowTailTransform = new RigidBodyTransform();
   private final RigidBodyTransform yArrowTailTransform = new RigidBodyTransform();

   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean isGizmoHoveredMouse = false;
   private boolean isBeingManipulated = false;
   /**
    * The main, source, true, base transform that this thing represents.
    */
   private final RigidBodyTransform transformToParent;
   private ReferenceFrame parentReferenceFrame;
   private ReferenceFrame gizmoFrame;
   private final RigidBodyTransform controlRingTransformToWorld = new RigidBodyTransform();
   private final FramePose3D controlRingPose = new FramePose3D();
   private final FramePose3D tempFramePose3D = new FramePose3D();
   private RDXFocusBasedCamera camera3D;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromKeyboardTransformationToWorld = new RigidBodyTransform();
   private ReferenceFrame keyboardTransformationFrame;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private double lastDistanceToCamera = -1.0;
   private final Plane3DMouseDragAlgorithm planeDragAlgorithm = new Plane3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm vrClockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private boolean showArrows = true;
   private boolean highlightingEnabled = true;
   private boolean isNewlyModifiedMouse;
   private final double translateSpeedFactor = 0.5;

   // VR moves the control-ring by calculating interection between the posivie-x direction ray from the controller and the ground.
   private ModelInstance lineModel;
   private final FrameLine3D vrPickRay = new FrameLine3D();
   private final FramePose3D vrPickRayPose = new FramePose3D();
   private final FramePose3D currentPlayArea = new FramePose3D();
   private final Plane3D currentPlayAreaPlane = new Plane3D();
   private final FrameVector3D controllerZAxisVector = new FrameVector3D();
   private final Point3D controllerZAxisProjectedToPlanePoint = new Point3D();
   private final Point3D planeRayIntesection = new Point3D();
   private Point3D intersectionStartPoint = new Point3D();
   private boolean isGizmoHoveredFromVR = false;
   private final Vector3D orientationDeterminationVector = new Vector3D();
   private final FramePose3D vrRayIntersectionWithGround = new FramePose3D();
   private double lineLength = 1.0;
   private final Color color = Color.ORANGE;
   private final RDXVRPickResult vrPickResult = new RDXVRPickResult();
   private boolean isVRTriggerDown = false;
   private boolean isVRTriggerClicked = false;
   private boolean isNewlyModifiedFromVR = false;
   private boolean sendSteps = false;
   private boolean isVRGrabbingGizmo = false;
   private double vrControllerYaw;
   private boolean isRingSelectedVR = false;
   private boolean isVRYawDragging = false;

   // Intersection calculator for mouse and vr
   private RayToControlRingIntersectionCalculator mouseRayToRingPickCalculator;
   private RayToControlRingIntersectionCalculator.CollisionType mouseCollisionType = RayToControlRingIntersectionCalculator.CollisionType.NONE;
   private RayToControlRingIntersectionCalculator vrRayToRingPickCalculator;
   private RayToControlRingIntersectionCalculator.CollisionType vrCollisionType = RayToControlRingIntersectionCalculator.CollisionType.NONE;

   // collision types
   private final RayToControlRingIntersectionCalculator.CollisionType hitCylinder = RayToControlRingIntersectionCalculator.CollisionType.CYLINDER;
   private final RayToControlRingIntersectionCalculator.CollisionType hitPositiveX = RayToControlRingIntersectionCalculator.CollisionType.POSITIVE_X;
   private final RayToControlRingIntersectionCalculator.CollisionType hitPositiveY = RayToControlRingIntersectionCalculator.CollisionType.POSITIVE_Y;
   private final RayToControlRingIntersectionCalculator.CollisionType hitNegativeX = RayToControlRingIntersectionCalculator.CollisionType.NEGATIVE_X;
   private final RayToControlRingIntersectionCalculator.CollisionType hitNegativeY = RayToControlRingIntersectionCalculator.CollisionType.NEGATIVE_Y;
   private final RayToControlRingIntersectionCalculator.CollisionType hitNone = RayToControlRingIntersectionCalculator.CollisionType.NONE;

   public RDXPathControlRingGizmo()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public RDXPathControlRingGizmo(RDXVRContext context)
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public RDXPathControlRingGizmo(ReferenceFrame parentReferenceFrame)
   {
      this.parentReferenceFrame = parentReferenceFrame;
      transformToParent = new RigidBodyTransform();
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
      keyboardTransformationFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                           transformFromKeyboardTransformationToWorld);
   }

   public void setParentFrame(ReferenceFrame parentReferenceFrame)
   {
      this.parentReferenceFrame = parentReferenceFrame;
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
   }

   private void buildLineMesh(RDXMultiColorMeshBuilder meshBuilder)
   {
      meshBuilder.addLine(0.0, 0.0, 0.0, lineLength, 0.0, 0.0, 0.005, color);
   }

   public void create(RDXFocusBasedCamera camera3D)
   {
      this.camera3D = camera3D;
      lineModel = RDXModelBuilder.buildModelInstance(this::buildLineMesh, "line");
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

      mouseRayToRingPickCalculator = new RayToControlRingIntersectionCalculator(xArrowTailTransform,
                                                                                yArrowTailTransform,
                                                                                discOuterRadius.get(),
                                                                                discInnerRadius.get(),
                                                                                discThickness.get(),
                                                                                arrowWidth.get(),
                                                                                arrowHeight.get(),
                                                                                arrowSpacing.get(),
                                                                                arrowTailWidthRatio.get(),
                                                                                arrowTailLengthRatio.get());

      vrRayToRingPickCalculator = new RayToControlRingIntersectionCalculator(xArrowTailTransform,
                                                                             yArrowTailTransform,
                                                                             discOuterRadius.get(),
                                                                             discInnerRadius.get(),
                                                                             discThickness.get(),
                                                                             arrowWidth.get(),
                                                                             arrowHeight.get(),
                                                                             arrowSpacing.get(),
                                                                             arrowTailWidthRatio.get(),
                                                                             arrowTailLengthRatio.get());

      recreateGraphics();
   }

   private Material createAlphaPaletteMaterial(float alpha)
   {
      Material material = new Material();
      material.set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      material.set(new BlendingAttribute(true, alpha));
      return material;
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
     {
        // Holding onto the right controller trigger button
        InputDigitalActionData clickTriggerData = controller.getClickTriggerActionData();
        isVRTriggerDown = clickTriggerData.bState();
        isVRTriggerClicked = isVRTriggerDown && clickTriggerData.bChanged();

        vrPickRay.setToZero(controller.getXForwardZUpControllerFrame());
        vrPickRay.getDirection().set(Axis3D.X);
        vrPickRay.changeFrame(ReferenceFrame.getWorldFrame());

        vrPickRayPose.setToZero(controller.getXForwardZUpControllerFrame());
        vrPickRayPose.changeFrame(ReferenceFrame.getWorldFrame());

        currentPlayArea.setToZero(vrContext.getTeleportFrameIHMCZUp());
        currentPlayArea.changeFrame(ReferenceFrame.getWorldFrame());

        currentPlayAreaPlane.getNormal().set(Axis3D.Z);
        currentPlayAreaPlane.getPoint().set(currentPlayArea.getPosition());

        currentPlayAreaPlane.intersectionWith(vrPickRay, planeRayIntesection);

        controllerZAxisVector.setIncludingFrame(controller.getXForwardZUpControllerFrame(), Axis3D.Z);
        controllerZAxisVector.changeFrame(ReferenceFrame.getWorldFrame());

        controllerZAxisProjectedToPlanePoint.set(planeRayIntesection);
        controllerZAxisProjectedToPlanePoint.add(controllerZAxisVector);
        currentPlayAreaPlane.orthogonalProjection(controllerZAxisProjectedToPlanePoint);

        orientationDeterminationVector.sub(controllerZAxisProjectedToPlanePoint, planeRayIntesection);

        vrRayIntersectionWithGround.setToZero(ReferenceFrame.getWorldFrame());
        vrRayIntersectionWithGround.getPosition().set(planeRayIntesection);
        EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X,
                                                                   orientationDeterminationVector,
                                                                   vrRayIntersectionWithGround.getOrientation());

        lineLength = vrPickRayPose.getPosition().distance(vrRayIntersectionWithGround.getPosition());
        RDXModelBuilder.rebuildMesh(lineModel.nodes.get(0), this::buildLineMesh);

        vrPickRayPose.get(tempTransform);
        LibGDXTools.toLibGDX(tempTransform, lineModel.transform);
        vrRayIntersectionWithGround.get(tempTransform);

        // vrRay intersected (collided) with this gizmo
        vrCollisionType = vrRayToRingPickCalculator.determineCollisionTypeFromRay(vrPickRay, controlRingTransformToWorld, showArrows);
        if (vrCollisionType != RayToControlRingIntersectionCalculator.CollisionType.NONE)
        {
           vrPickResult.setDistanceToControllerPickPoint(vrRayToRingPickCalculator.getClosestCollisionDistance());
           vrContext.addPickResult(RobotSide.RIGHT, vrPickResult);
           isGizmoHoveredFromVR = true;
           updateMaterialHighlighting();
        }
        else
        {
           isGizmoHoveredFromVR = false;
        }

        if (isVRTriggerClicked)
        {
           isRingSelectedVR = getHollowCylinderPickSelectedVR();
        }

        if (isRingSelectedVR)
        {
           vrContext.addPickResult(RobotSide.RIGHT, vrPickResult);
        }

     });
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
     {
        // Holding onto the right controller trigger button
        InputDigitalActionData clickTriggerData = controller.getClickTriggerActionData();
        InputDigitalActionData bButton = controller.getBButtonActionData();
        isVRTriggerDown = clickTriggerData.bState();
        isVRTriggerClicked = isVRTriggerDown && clickTriggerData.bChanged();
        boolean isVRBButtonDown = bButton.bState();

        // Grabbing the ring with vrController
        if (!isVRTriggerDown)
        {
           isVRGrabbingGizmo = false;
        }

        if (isRingHoveredFromVR() && isVRBButtonDown)
        {
           if (!isVRYawDragging)
           {
              // start point of yaw-dragging by VR
              isVRYawDragging = true;
              vrClockFaceDragAlgorithm.reset();
           }
        }

        if (isVRYawDragging && !isVRBButtonDown)
           isVRYawDragging = false;

        if (isVRYawDragging)
        {
           // yaw drag
           if (vrClockFaceDragAlgorithm.calculate(vrPickRay, vrRayToRingPickCalculator.getClosestCollision(), Axis3D.Z, controlRingPose))
           {
              isNewlyModifiedFromVR = true;
              tempFramePose3D.setToZero(gizmoFrame);
              tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
              vrClockFaceDragAlgorithm.getMotion().transform(tempFramePose3D.getOrientation());
              tempFramePose3D.changeFrame(parentReferenceFrame);
              tempFramePose3D.get(transformToParent);
           }
        }


        if (isVRGrabbingGizmo)
        {
           isNewlyModifiedFromVR = true;
           // translating gizmo with vr-ray
           Vector3D planarMotion = new Vector3D();          // This represents vector from previous intersection to updated (moved) intersection point.
           planarMotion.sub(vrRayIntersectionWithGround.getPosition(), intersectionStartPoint);
           // tempFramePose3D gets updated from all the processes and eventually updates controlRingPose
           tempFramePose3D.setToZero(gizmoFrame);
           tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
           tempFramePose3D.getPosition().add(planarMotion);
           tempFramePose3D.changeFrame(parentReferenceFrame);
           tempFramePose3D.get(transformToParent);
           // also update closestCollision with the vector
           vrRayToRingPickCalculator.getClosestCollision().add(planarMotion);
           // update previous vrIntersectionPoint with current intersectionPoint.
           intersectionStartPoint = new Point3D(vrRayIntersectionWithGround.getPosition());
        }
        // Initial point user starts grabbing the ring.
        else if (isVRTriggerDown && isGizmoHoveredFromVR && vrCollisionType == hitCylinder)
        {
           intersectionStartPoint = new Point3D(vrRayIntersectionWithGround.getPosition());
           isVRGrabbingGizmo = true;
           isNewlyModifiedFromVR = true;
           vrControllerYaw = controller.getXForwardZUpPose().getYaw();
        }
        else if (!isVRYawDragging())
        {
           isNewlyModifiedFromVR = false;
        }
     });
      updateTransforms();
   }

   public boolean isSendSteps()
   {
      return sendSteps;
   }

   public void setSendSteps(boolean send)
   {
      sendSteps = send;
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      updateTransforms();

      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(ImGuiMouseButton.Right);

      if (!translateDragData.isDragging() && !yawDragData.isDragging())
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         mouseCollisionType = mouseRayToRingPickCalculator.determineCollisionTypeFromRay(pickRay, controlRingTransformToWorld, showArrows);
         updateMaterialHighlighting();
      }
      if (mouseCollisionType != RayToControlRingIntersectionCalculator.CollisionType.NONE)
      {
         pickResult.setDistanceToCamera(mouseRayToRingPickCalculator.getClosestCollisionDistance());
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

      int yawMouseButton = ImGuiMouseButton.Right;
      ImGuiMouseDragData translateDragData = input.getMouseDragData(ImGuiMouseButton.Left);
      ImGuiMouseDragData yawDragData = input.getMouseDragData(yawMouseButton);

      isNewlyModifiedMouse = false;
      isGizmoHoveredMouse = input.isWindowHovered() && pickResult == input.getClosestPick();
      boolean isRingHoveredMouse = isGizmoHoveredMouse && mouseCollisionType == RayToControlRingIntersectionCalculator.CollisionType.CYLINDER;
      boolean leftButtonDown = ImGui.isMouseDown(ImGuiMouseButton.Left);
      boolean rightButtonDown = ImGui.isMouseDown(yawMouseButton);

      if (allowUserInput)
      {
         if (isRingHoveredMouse)
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

         isBeingManipulated =
               (yawDragData.getObjectBeingDragged() == this && rightButtonDown) || (translateDragData.getObjectBeingDragged() == this && leftButtonDown);
         if (isBeingManipulated)
         {
            isNewlyModifiedMouse = true;
            Line3DReadOnly pickRay = input.getPickRayInWorld();

            if (translateDragData.isDragging())
            {
               Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, mouseRayToRingPickCalculator.getClosestCollision(), Axis3D.Z);
               tempFramePose3D.setToZero(gizmoFrame);
               tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
               tempFramePose3D.getPosition().add(planarMotion);
               tempFramePose3D.changeFrame(parentReferenceFrame);
               tempFramePose3D.get(transformToParent);
               mouseRayToRingPickCalculator.getClosestCollision().add(planarMotion);
            }
            else // yaw dragging
            {
               if (clockFaceDragAlgorithm.calculate(pickRay, mouseRayToRingPickCalculator.getClosestCollision(), Axis3D.Z, controlRingPose))
               {
                  tempFramePose3D.setToZero(gizmoFrame);
                  tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
                  clockFaceDragAlgorithm.getMotion().transform(tempFramePose3D.getOrientation());
                  tempFramePose3D.changeFrame(parentReferenceFrame);
                  tempFramePose3D.get(transformToParent);
               }
            }
         }

         // keyboard based controls
         boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
         boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
         boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
         boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
         boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
         isNewlyModifiedMouse |= anyArrowHeld;
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
            keyboardTransformationFrame.update();
            tempFramePose3D.setToZero(keyboardTransformationFrame);

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

   private void updateMaterialHighlighting()
   {
      RayToControlRingIntersectionCalculator.CollisionType cylinder = RayToControlRingIntersectionCalculator.CollisionType.CYLINDER;
      RayToControlRingIntersectionCalculator.CollisionType positiveX = RayToControlRingIntersectionCalculator.CollisionType.POSITIVE_X;
      RayToControlRingIntersectionCalculator.CollisionType positiveY = RayToControlRingIntersectionCalculator.CollisionType.POSITIVE_Y;
      RayToControlRingIntersectionCalculator.CollisionType negativeX = RayToControlRingIntersectionCalculator.CollisionType.NEGATIVE_X;
      RayToControlRingIntersectionCalculator.CollisionType negativeY = RayToControlRingIntersectionCalculator.CollisionType.NEGATIVE_Y;

      boolean prior = highlightingEnabled && (isGizmoHoveredMouse || isGizmoHoveredFromVR);
      discModel.setMaterial(prior && (mouseCollisionType == cylinder || vrCollisionType == cylinder) ? highlightedMaterial : normalMaterial);
      positiveXArrowModel.setMaterial(prior && (mouseCollisionType == positiveX || vrCollisionType == positiveX) ? highlightedMaterial : normalMaterial);
      positiveYArrowModel.setMaterial(prior && (mouseCollisionType == positiveY || vrCollisionType == positiveY) ? highlightedMaterial : normalMaterial);
      negativeXArrowModel.setMaterial(prior && (mouseCollisionType == negativeX || vrCollisionType == negativeX) ? highlightedMaterial : normalMaterial);
      negativeYArrowModel.setMaterial(prior && (mouseCollisionType == negativeY || vrCollisionType == negativeY) ? highlightedMaterial : normalMaterial);
   }

   public ImGuiPanel createTunerPanel(String name)
   {
      return new ImGuiPanel("Footstep Ring Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
   }

   public void renderImGuiWidgets()
   {

   }

   public void renderImGuiTuner()
   {
      ImGui.text("Use the right mouse button to manipulate the widget.");

      if (ImGui.button("Reset"))
      {
         transformToParent.setToZero();
      }

      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc outer radius"), discOuterRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc inner radius"), discInnerRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc thickness"), discThickness.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow width"), arrowWidth.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow height"), arrowHeight.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow spacing"), arrowSpacing.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow tail width ratio"), arrowTailWidthRatio.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow tail length ratio"), arrowTailLengthRatio.getData(), 0.001f, 0.0f, 1000.0f);
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
      lineModel.getRenderables(renderables, pool);
   }

   public void getVRLineRenderable(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      lineModel.getRenderables(renderables, pool);
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

   public boolean getAnyPartPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType != hitNone;
   }

   public boolean getAnyPartPickSelectedVR()
   {
      return isGizmoHoveredFromVR  && vrCollisionType != hitNone;
   }

   public boolean getAnyArrowPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType != hitNone && mouseCollisionType != hitCylinder;
   }

   public boolean getAnyArrowPickSelectedVR()
   {
      return isGizmoHoveredFromVR  && vrCollisionType != hitNone && vrCollisionType != hitCylinder;
   }

   // CYLINDER - MOUSE
   public boolean getHollowCylinderPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType == hitCylinder;
   }
   // CYLINDER - VR
   public boolean getHollowCylinderPickSelectedVR()
   {
      return isGizmoHoveredFromVR && vrCollisionType == hitCylinder;
   }

   // POSITIVE X - MOUSE
   public boolean getPositiveXArrowPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType == hitPositiveX;
   }
   // POSITIVE X - VR
   public boolean getPositiveXArrowPickSelectedVR()
   {
      return isGizmoHoveredFromVR && vrCollisionType == hitPositiveX;
   }

   // POSITIVE Y - MOUSE
   public boolean getPositiveYArrowPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType == hitPositiveY;
   }
   // POSITIVE Y - VR
   public boolean getPositiveYArrowPickSelectedVR()
   {
      return isGizmoHoveredFromVR && vrCollisionType == hitPositiveY;
   }

   // NEGATIVE X - MOUSE
   public boolean getNegativeXArrowPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType == hitNegativeX;
   }
   // NEGATIVE X - VR
   public boolean getNegativeXArrowPickSelectedVR()
   {
      return isGizmoHoveredFromVR && isVRTriggerClicked && vrCollisionType == hitNegativeX;
   }

   // NEGATIVE Y - MOUSE
   public boolean getNegativeYArrowPickSelectedMouse()
   {
      return isGizmoHoveredMouse && mouseCollisionType == hitNegativeY;
   }
   // NEGATIVE Y - VR
   public boolean getNegativeYArrowPickSelectedVR()
   {
      return isGizmoHoveredFromVR && isVRTriggerClicked && vrCollisionType == hitNegativeY;
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

   public boolean isNewlyModifiedMouse()
   {
      return isNewlyModifiedMouse;
   }

   public boolean isNewlyModifiedVR()
   {
      return isNewlyModifiedFromVR;
   }

   public boolean isGizmoHoveredMouse()
   {
      return isGizmoHoveredMouse;
   }

   public boolean isVRTriggerDown()
   {
      return isVRTriggerDown;
   }

   public boolean isGizmoGrabbedFromVR()
   {
      return isVRTriggerDown && isGizmoHoveredFromVR;
   }

   public boolean isRingHoveredFromVR()
   {
      return isGizmoHoveredFromVR && vrCollisionType == hitCylinder;
   }

   public boolean isVRTriggerClicked()
   {
      return isVRTriggerClicked;
   }

   public boolean isVRYawDragging()
   {
      return isVRYawDragging;
   }
}
