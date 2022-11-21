package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
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
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMeshBuilder;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

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
   private boolean isGizmoHoveredMouse = false;
   private boolean isBeingManipulatedMouse = false;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private final DiscreteArrowRayIntersection arrowIntersection = new DiscreteArrowRayIntersection();
   private final FramePose3D framePose3D = new FramePose3D();
   // This pose tracks the changes to the gizmo pose and is used to update the pose of this gizmo, i.e transformToWorld, in UpdateTransforms()
   private final FramePose3D tempFramePose3D = new FramePose3D();
   /** The main, source, true, base transform that this thing represents. */
   private RigidBodyTransform transformToParent;
   private ReferenceFrame parentReferenceFrame;
   private ReferenceFrame gizmoFrame;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private final Line3DMouseDragAlgorithm lineDragAlgorithm = new Line3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private RDXFocusBasedCamera camera3D;
   private final FramePose3D keyboardAdjustmentPose3D = new FramePose3D();
   private ModifiableReferenceFrame keyboardTransformationFrameInWorld;
   private final Point3D cameraPosition = new Point3D();
   private double distanceToCamera;
   private double lastDistanceToCamera = -1.0;
   private final double translateSpeedFactor = 0.5;
   private boolean queuePopupToOpen = false;
   private final Random random = new Random();

   // VR moves the control-ring by calculating interection between the posivie-x direction ray from the controller and the ground.
   // VR variables to interact with gizmo.
   private ModelInstance lineModel;
   private final FrameLine3D vrPickRay = new FrameLine3D();
   private final FramePose3D vrPickRayPose = new FramePose3D();
   private final FramePose3D currentPlayArea = new FramePose3D();
   private final Plane3D currentPlayAreaPlane = new Plane3D();
   private final FrameVector3D controllerZAxisVector = new FrameVector3D();
   private final Point3D controllerZAxisProjectedToPlanePoint = new Point3D();
   private final Point3D planeRayIntesection = new Point3D();
   private Point3D intersectionStartPoint = new Point3D();
   private boolean isGizmoHoveredVR = false;
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

   // Intersection calculator for vr.
   private RayToGizmoIntersectionCalculator mouseRayIntersectionCalculator;
   private RayToGizmoIntersectionCalculator vrRayIntersectionCalculator;

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

   private void initialize(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this.parentReferenceFrame = gizmoFrame.getParent();
      this.transformToParent = gizmoTransformToParentFrameToModify;
      this.gizmoFrame = gizmoFrame;
      keyboardTransformationFrameInWorld = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
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
      }


      // initialize ray-intersection calculators here.
      mouseRayIntersectionCalculator = new RayToGizmoIntersectionCalculator(torusRadius.get(),
                                                                            torusCameraSize.get(),
                                                                            torusTubeRadiusRatio.get(),
                                                                            arrowLengthRatio.get(),
                                                                            arrowHeadBodyLengthRatio.get(),
                                                                            arrowHeadBodyRadiusRatio.get(),
                                                                            arrowSpacingFactor.get());

      vrRayIntersectionCalculator = new RayToGizmoIntersectionCalculator(torusRadius.get(),
                                                                         torusCameraSize.get(),
                                                                         torusTubeRadiusRatio.get(),
                                                                         arrowLengthRatio.get(),
                                                                         arrowHeadBodyLengthRatio.get(),
                                                                         arrowHeadBodyRadiusRatio.get(),
                                                                         arrowSpacingFactor.get());

      recreateGraphics();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      updateTransforms();

      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      // Here we are trying to avoid unecessary computation in collision calculation by filtering out
      // some common scenarios where we don't need to calculate the pick, which can be expensive
      if (isWindowHovered && (!manipulationDragData.isDragging() || manipulationDragData.getDragJustStarted()))
      {
         // This part is happening when the user could presumably start a drag
         // on this gizmo at any time

         Line3DReadOnly pickRay = input.getPickRayInWorld();
         mouseRayIntersectionCalculator.determineCollisionSelectionFromRay(pickRay, transformToWorld, tempTransform, torusModels, arrowModels);

         if (mouseRayIntersectionCalculator.getClosestCollisionSelection()  != null)
         {
            pickResult.setDistanceToCamera(mouseRayIntersectionCalculator.getClosestCollisionDistance());
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      updateTransforms();

      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      isGizmoHoveredMouse = input.isWindowHovered() && pickResult == input.getClosestPick();

      if (isGizmoHoveredMouse && ImGui.getMouseClickedCount(ImGuiMouseButton.Right) == 1)
      {
         queuePopupToOpen = true;
      }

      updateMaterialHighlighting();

      if (isGizmoHoveredMouse && manipulationDragData.getDragJustStarted())
      {
         clockFaceDragAlgorithm.reset();
         manipulationDragData.setObjectBeingDragged(this);
      }

      isBeingManipulatedMouse = manipulationDragData.getObjectBeingDragged() == this;
      if (isBeingManipulatedMouse)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         SixDoFSelection mouseSelection = mouseRayIntersectionCalculator.getClosestCollisionSelection();

         if (mouseSelection.isLinear())
         {
            Vector3DReadOnly linearMotion = lineDragAlgorithm.calculate(pickRay,
                                                                        mouseRayIntersectionCalculator.getClosestCollision(),
                                                                        axisRotations.get(mouseSelection.toAxis3D()),
                                                                        transformToWorld);

            tempFramePose3D.setToZero(gizmoFrame);
            tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
            tempFramePose3D.getPosition().add(linearMotion);
            tempFramePose3D.changeFrame(parentReferenceFrame);
            tempFramePose3D.get(transformToParent);
            mouseRayIntersectionCalculator.getClosestCollision().add(linearMotion);
         }
         else if (mouseSelection.isAngular())
         {
            if (clockFaceDragAlgorithm.calculate(pickRay,
                                                 mouseRayIntersectionCalculator.getClosestCollision(),
                                                 axisRotations.get(mouseSelection.toAxis3D()),
                                                 transformToWorld))
            {
               tempFramePose3D.setToZero(gizmoFrame);
               tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
               clockFaceDragAlgorithm.getMotion().transform(tempFramePose3D.getOrientation());
               tempFramePose3D.changeFrame(parentReferenceFrame);
               tempFramePose3D.get(transformToParent);
            }
         }
      }

      // Use mouse wheel to yaw when ctrl key is held
      if (ImGui.getIO().getKeyCtrl() && input.getMouseWheelDelta() != 0.0f)
      {
         float deltaScroll = input.getMouseWheelDelta();
         tempFramePose3D.setToZero(gizmoFrame);
         tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
         // Add some noise to not get stuck in discrete space
         double noise = random.nextDouble() * 0.005;
         double speed = 0.012 + noise;
         tempFramePose3D.getOrientation().appendYawRotation(Math.signum(deltaScroll) * speed * Math.PI);
         tempFramePose3D.changeFrame(parentReferenceFrame);
         tempFramePose3D.get(transformToParent);
      }

      // keyboard based controls
      boolean upArrowHeld = ImGui.isKeyDown(ImGuiTools.getUpArrowKey());
      boolean downArrowHeld = ImGui.isKeyDown(ImGuiTools.getDownArrowKey());
      boolean leftArrowHeld = ImGui.isKeyDown(ImGuiTools.getLeftArrowKey());
      boolean rightArrowHeld = ImGui.isKeyDown(ImGuiTools.getRightArrowKey());
      boolean anyArrowHeld = upArrowHeld || downArrowHeld || leftArrowHeld || rightArrowHeld;
      if (anyArrowHeld) // only the arrow keys do the moving
      {
         keyboardTransformationFrameInWorld.getTransformToParent().setToZero();
         keyboardTransformationFrameInWorld.getTransformToParent().getRotation().setToYawOrientation(camera3D.getFocusPointPose().getYaw());
         keyboardTransformationFrameInWorld.getReferenceFrame().update();
         keyboardAdjustmentPose3D.setToZero(gizmoFrame);
         keyboardAdjustmentPose3D.changeFrame(keyboardTransformationFrameInWorld.getReferenceFrame());

         boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
         boolean altHeld = ImGui.getIO().getKeyAlt();
         boolean shiftHeld = ImGui.getIO().getKeyShift();
         double deltaTime = Gdx.graphics.getDeltaTime();
         if (altHeld) // orientation
         {
            double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
            if (upArrowHeld) // pitch +
            {
               keyboardAdjustmentPose3D.getOrientation().appendPitchRotation(amount);
            }
            if (downArrowHeld) // pitch -
            {
               keyboardAdjustmentPose3D.getOrientation().appendPitchRotation(-amount);
            }
            if (rightArrowHeld && !ctrlHeld) // roll +
            {
               keyboardAdjustmentPose3D.getOrientation().appendRollRotation(amount);
            }
            if (leftArrowHeld && !ctrlHeld) // roll -
            {
               keyboardAdjustmentPose3D.getOrientation().appendRollRotation(-amount);
            }
            if (leftArrowHeld && ctrlHeld) // yaw +
            {
               keyboardAdjustmentPose3D.getOrientation().appendYawRotation(amount);
            }
            if (rightArrowHeld && ctrlHeld) // yaw -
            {
               keyboardAdjustmentPose3D.getOrientation().appendYawRotation(-amount);
            }
         }
         else // translation
         {
            double amount = deltaTime * (shiftHeld ? 0.05 : 0.4);
            distanceToCamera = cameraPosition.distance(framePose3D.getPosition());
            if (upArrowHeld && !ctrlHeld) // x +
            {
               keyboardAdjustmentPose3D.getPosition().addX(getTranslateSpeedFactor() * amount);
            }
            if (downArrowHeld && !ctrlHeld) // x -
            {
               keyboardAdjustmentPose3D.getPosition().subX(getTranslateSpeedFactor() * amount);
            }
            if (leftArrowHeld) // y +
            {
               keyboardAdjustmentPose3D.getPosition().addY(getTranslateSpeedFactor() * amount);
            }
            if (rightArrowHeld) // y -
            {
               keyboardAdjustmentPose3D.getPosition().subY(getTranslateSpeedFactor() * amount);
            }
            if (upArrowHeld && ctrlHeld) // z +
            {
               keyboardAdjustmentPose3D.getPosition().addZ(getTranslateSpeedFactor() * amount);
            }
            if (downArrowHeld && ctrlHeld) // z -
            {
               keyboardAdjustmentPose3D.getPosition().subZ(getTranslateSpeedFactor() * amount);
            }
         }

         keyboardAdjustmentPose3D.changeFrame(parentReferenceFrame);
         keyboardAdjustmentPose3D.get(transformToParent);
      }

      // after things have been modified, update the derivative stuff
      updateTransforms();

      if (resizeAutomatically.get())
      {
         LibGDXTools.toEuclid(camera3D.position, cameraPosition);
         distanceToCamera = cameraPosition.distance(framePose3D.getPosition());
         if (lastDistanceToCamera != distanceToCamera)
         {
            lastDistanceToCamera = distanceToCamera;
            recreateGraphics();
            updateTransforms();
         }
      }
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
        vrRayIntersectionCalculator.determineCollisionSelectionFromRay(vrPickRay, transformToWorld, tempTransform, torusModels, arrowModels);
//        vrCollisionType = vrRayToRingPickCalculator.determineCollisionTypeFromRay(vrPickRay, transformToWorld, showArrows);
        if (vrRayIntersectionCalculator.getClosestCollisionSelection() != null)
        {
           vrPickResult.setDistanceToControllerPickPoint(vrRayIntersectionCalculator.getClosestCollisionDistance());
           vrContext.addPickResult(RobotSide.RIGHT, vrPickResult);
           isGizmoHoveredVR = true;
           updateMaterialHighlighting();
        }
        else
        {
           isGizmoHoveredVR = false;
        }

//        if (isVRTriggerClicked)
//        {
//           isRingSelectedVR = getHollowCylinderPickSelectedVR();
//        }

        if (isRingSelectedVR)
        {
           vrContext.addPickResult(RobotSide.RIGHT, vrPickResult);
        }
     });
   }

   /*
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
   
    */



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
         if (ImGui.menuItem("Cancel"))
            ImGui.closeCurrentPopup();
         ImGui.endPopup();
      }
   }

   /** Call this instead of process3DViewInput if the gizmo is deactivated. */
   public void updateTransforms()
   {
      gizmoFrame.update();
      for (Axis3D axis : Axis3D.values)
      {
         framePose3D.setToZero(gizmoFrame);
         framePose3D.getOrientation().set(axisRotations.get(axis));
         framePose3D.changeFrame(ReferenceFrame.getWorldFrame());
         framePose3D.get(tempTransform);
         LibGDXTools.toLibGDX(tempTransform, arrowModels[axis.ordinal()].getOrCreateModelInstance().transform);
         LibGDXTools.toLibGDX(tempTransform, torusModels[axis.ordinal()].getOrCreateModelInstance().transform);
      }
      tempFramePose3D.setToZero(gizmoFrame);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose3D.get(transformToWorld);
   }

   private void updateMaterialHighlighting()
   {

      SixDoFSelection mouseSelection = mouseRayIntersectionCalculator.getClosestCollisionSelection();
      boolean priorMouse = (isGizmoHoveredMouse || isBeingManipulatedMouse) && mouseSelection != null;
      // could only do this when selection changed
      for (Axis3D axis : Axis3D.values)
      {
         if (priorMouse && mouseSelection.isAngular() && mouseSelection.toAxis3D() == axis)
         {
            torusModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            torusModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }

         if (priorMouse && mouseSelection.isLinear() && mouseSelection.toAxis3D() == axis)
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

      ImGui.checkbox("Resize based on camera distance", resizeAutomatically);

      boolean proportionsChanged = false;
      ImGui.pushItemWidth(100.00f);
      if (resizeAutomatically.get())
         proportionsChanged |= ImGui.dragFloat(labels.get("Torus camera size"), torusCameraSize.getData(), 0.001f);
      else
         proportionsChanged |= ImGui.dragFloat(labels.get("Torus radius"), torusRadius.getData(), 0.001f);
      ImGui.popItemWidth();

      if (ImGui.collapsingHeader(labels.get("Advanced")))
      {
         if (ImGui.button("Set to zero in parent frame"))
         {
            transformToParent.setToZero();
         }

         ImGui.pushItemWidth(100.00f);
         proportionsChanged |= ImGui.dragFloat(labels.get("Torus tube radius ratio"), torusTubeRadiusRatio.getData(), 0.001f);
         proportionsChanged |= ImGui.dragFloat(labels.get("Arrow length ratio"), arrowLengthRatio.getData(), 0.05f);
         proportionsChanged |= ImGui.dragFloat(labels.get("Arrow head body length ratio"), arrowHeadBodyLengthRatio.getData(), 0.05f);
         proportionsChanged |= ImGui.dragFloat(labels.get("Arrow head body radius ratio"), arrowHeadBodyRadiusRatio.getData(), 0.05f);
         proportionsChanged |= ImGui.dragFloat(labels.get("Arrow spacing factor"), arrowSpacingFactor.getData(), 0.05f);
         ImGui.popItemWidth();
      }

      if (proportionsChanged)
         recreateGraphics();

      ImGui.text("Drag using the left mouse button to manipulate the gizmo.");

      updateTransforms();
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

      mouseRayIntersectionCalculator.updateGizmoDimensions(torusRadius.get(),
                                                           torusCameraSize.get(),
                                                           torusTubeRadiusRatio.get(),
                                                           arrowLengthRatio.get(),
                                                           arrowHeadBodyLengthRatio.get(),
                                                           arrowHeadBodyRadiusRatio.get(),
                                                           arrowSpacingFactor.get());
      vrRayIntersectionCalculator.updateGizmoDimensions(torusRadius.get(),
                                                           torusCameraSize.get(),
                                                           torusTubeRadiusRatio.get(),
                                                           arrowLengthRatio.get(),
                                                           arrowHeadBodyLengthRatio.get(),
                                                           arrowHeadBodyRadiusRatio.get(),
                                                           arrowSpacingFactor.get());
      updateMaterialHighlighting();

      for (Axis3D axis : Axis3D.values)
      {
         arrowModels[axis.ordinal()].invalidateMesh();
         torusModels[axis.ordinal()].invalidateMesh();
      }
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

   private void buildLineMesh(RDXMultiColorMeshBuilder meshBuilder)
   {
      meshBuilder.addLine(0.0, 0.0, 0.0, lineLength, 0.0, 0.0, 0.005, color);
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
}
