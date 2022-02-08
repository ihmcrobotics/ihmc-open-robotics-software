package us.ihmc.gdx.ui.gizmo;

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
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.mesh.GDXMeshBuilder;
import us.ihmc.gdx.mesh.GDXMeshDataInterpreter;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.gdx.ui.gizmo.GDXGizmoTools.AXIS_COLORS;
import static us.ihmc.gdx.ui.gizmo.GDXGizmoTools.AXIS_SELECTED_COLORS;

public class GDXPose3DGizmo implements RenderableProvider
{
   private final ImFloat torusRadius = new ImFloat(0.5f);
   private final ImFloat torusCameraSize = new ImFloat(0.067f);
   private final ImFloat torusTubeRadiusRatio = new ImFloat(0.074f);
   private final ImFloat arrowLengthRatio = new ImFloat(0.431f);
   private final ImFloat arrowHeadBodyLengthRatio = new ImFloat(0.480f);
   private final ImFloat arrowHeadBodyRadiusRatio = new ImFloat(2.0f);
   private final ImFloat arrowSpacingFactor = new ImFloat(2.22f);
   private final ImBoolean resizeAutomatically = new ImBoolean(true);
   private double animationSpeed = 0.25 * Math.PI;
   private double arrowBodyRadius;
   private double arrowLength;
   private double arrowBodyLength;
   private double arrowHeadRadius;
   private double arrowHeadLength;
   private double arrowSpacing;
   private final Material[] normalMaterials = new Material[3];
   private final Material[] highlightedMaterials = new Material[3];
   private final Axis3DRotations axisRotations = new Axis3DRotations();
   private final DynamicGDXModel[] arrowModels = new DynamicGDXModel[3];
   private final DynamicGDXModel[] torusModels = new DynamicGDXModel[3];
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private final DiscreteArrowRayIntersection arrowIntersection = new DiscreteArrowRayIntersection();
   private final FramePose3D framePose3D = new FramePose3D();
   private final FramePose3D tempFramePose3D = new FramePose3D();
   /** The main, source, true, base transform that this thing represents. */
   private final RigidBodyTransform transformToParent;
   private final ReferenceFrame parentReferenceFrame;
   private final ReferenceFrame gizmoFrame;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private final Line3DMouseDragAlgorithm lineDragAlgorithm = new Line3DMouseDragAlgorithm();
   private final ClockFaceRotation3DMouseDragAlgorithm clockFaceDragAlgorithm = new ClockFaceRotation3DMouseDragAlgorithm();
   private FocusBasedGDXCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double lastDistanceToCamera = -1.0;

   public GDXPose3DGizmo()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public GDXPose3DGizmo(ReferenceFrame parentReferenceFrame)
   {
      this.parentReferenceFrame = parentReferenceFrame;
      transformToParent = new RigidBodyTransform();
      gizmoFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentReferenceFrame, transformToParent);
   }

   public GDXPose3DGizmo(ReferenceFrame gizmoFrame, RigidBodyTransform gizmoTransformToParentFrameToModify)
   {
      this.parentReferenceFrame = gizmoFrame.getParent();
      this.transformToParent = gizmoTransformToParentFrameToModify;
      this.gizmoFrame = gizmoFrame;
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      this.camera3D = camera3D;

      for (Axis3D axis : Axis3D.values)
      {
         Color color = AXIS_COLORS[axis.ordinal()];
         normalMaterials[axis.ordinal()] = new Material();
         normalMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
         normalMaterials[axis.ordinal()].set(new BlendingAttribute(true, color.a));
         highlightedMaterials[axis.ordinal()] = new Material();
         highlightedMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
         highlightedMaterials[axis.ordinal()].set(new BlendingAttribute(true, AXIS_SELECTED_COLORS[axis.ordinal()].a));
         arrowModels[axis.ordinal()] = new DynamicGDXModel();
         arrowModels[axis.ordinal()].setMesh(meshBuilder ->
         {
            // Euclid cylinders are defined from the center, but mesh builder defines them from the bottom
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing), color);
            meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing + arrowBodyLength), color);
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, -0.5 * arrowSpacing), FLIP_180, color);
         });
         torusModels[axis.ordinal()] = new DynamicGDXModel();
         int resolution = 25;
         torusModels[axis.ordinal()].setMesh(
            meshBuilder -> meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), resolution, color));
      }

      recreateGraphics();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      updateTransforms();

      boolean rightMouseDragging = input.isDragging(ImGuiMouseButton.Right);
      boolean rightMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Right);
      boolean isWindowHovered = ImGui.isWindowHovered();

      if (isWindowHovered && !rightMouseDragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (rightMouseDown && closestCollisionSelection != null)
         {
            clockFaceDragAlgorithm.reset();
         }
      }
      if (rightMouseDragging && closestCollisionSelection != null)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (closestCollisionSelection.isLinear())
         {
            Vector3DReadOnly linearMotion = lineDragAlgorithm.calculate(pickRay,
                                                                        closestCollision,
                                                                        axisRotations.get(closestCollisionSelection.toAxis3D()), transformToWorld);

            tempFramePose3D.setToZero(gizmoFrame);
            tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
            tempFramePose3D.getPosition().add(linearMotion);
            tempFramePose3D.changeFrame(parentReferenceFrame);
            tempFramePose3D.get(transformToParent);
            closestCollision.add(linearMotion);
         }
         else if (closestCollisionSelection.isAngular())
         {
            if (clockFaceDragAlgorithm.calculate(pickRay,
                                                 closestCollision,
                                                 axisRotations.get(closestCollisionSelection.toAxis3D()), transformToWorld))
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
      if (anyArrowHeld) // only the arrow keys do the moving
      {
         boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
         boolean altHeld = ImGui.getIO().getKeyAlt();
         boolean shiftHeld = ImGui.getIO().getKeyShift();
         double deltaTime = Gdx.graphics.getDeltaTime();
         if (altHeld) // orientation
         {
            double amount = deltaTime * (shiftHeld ? 0.2 : 1.0);
            if (upArrowHeld) // pitch +
            {
               transformToParent.getRotation().appendPitchRotation(amount);
            }
            if (downArrowHeld) // pitch -
            {
               transformToParent.getRotation().appendPitchRotation(-amount);
            }
            if (rightArrowHeld && !ctrlHeld) // roll +
            {
               transformToParent.getRotation().appendRollRotation(amount);
            }
            if (leftArrowHeld && !ctrlHeld) // roll -
            {
               transformToParent.getRotation().appendRollRotation(-amount);
            }
            if (leftArrowHeld && ctrlHeld) // yaw +
            {
               transformToParent.getRotation().appendYawRotation(amount);
            }
            if (rightArrowHeld && ctrlHeld) // yaw -
            {
               transformToParent.getRotation().appendYawRotation(-amount);
            }
         }
         else // translation
         {
            double amount = deltaTime * (shiftHeld ? 0.05 : 0.4);
            if (upArrowHeld && !ctrlHeld) // x +
            {
               transformToParent.getTranslation().addX(amount);
            }
            if (downArrowHeld && !ctrlHeld) // x -
            {
               transformToParent.getTranslation().subX(amount);
            }
            if (leftArrowHeld) // y +
            {
               transformToParent.getTranslation().addY(amount);
            }
            if (rightArrowHeld) // y -
            {
               transformToParent.getTranslation().subY(amount);
            }
            if (upArrowHeld && ctrlHeld) // z +
            {
               transformToParent.getTranslation().addZ(amount);
            }
            if (downArrowHeld && ctrlHeld) // z -
            {
               transformToParent.getTranslation().subZ(amount);
            }
         }
      }

      // after things have been modified, update the derivative stuff
      updateTransforms();

      if (resizeAutomatically.get())
      {
         GDXTools.toEuclid(camera3D.position, cameraPosition);
         double distanceToCamera = cameraPosition.distance(framePose3D.getPosition());
         if (lastDistanceToCamera != distanceToCamera)
         {
            lastDistanceToCamera = distanceToCamera;
            recreateGraphics();
            updateTransforms();
         }
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
         GDXTools.toGDX(tempTransform, arrowModels[axis.ordinal()].getOrCreateModelInstance().transform);
         GDXTools.toGDX(tempTransform, torusModels[axis.ordinal()].getOrCreateModelInstance().transform);
      }
      tempFramePose3D.setToZero(gizmoFrame);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose3D.get(transformToWorld);
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      closestCollisionSelection = null;
      double closestCollisionDistance = Double.POSITIVE_INFINITY;

      // Optimization: Do one large sphere collision to avoid completely far off picks
      boundingSphereIntersection.setup(1.5 * torusRadius.get(), transformToWorld);
      if (boundingSphereIntersection.intersect(pickRay))
      {
         // collide tori
         for (Axis3D axis : Axis3D.values)
         {
            GDXTools.toEuclid(torusModels[axis.ordinal()].getOrCreateModelInstance().transform, tempTransform);
            // TODO: Only setup when shape changes?
            torusIntersection.setupTorus(torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), tempTransform);
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
            GDXTools.toEuclid(arrowModels[axis.ordinal()].getOrCreateModelInstance().transform, tempTransform);

            for (RobotSide side : RobotSide.values)
            {
               double zOffset = side.negateIfRightSide(0.5 * arrowSpacing + 0.5 * arrowBodyLength);
               // TODO: Only setup when shape changes?
               arrowIntersection.setupShapes(arrowBodyLength, arrowBodyRadius, arrowHeadRadius, arrowHeadLength, zOffset, tempTransform);
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

      updateMaterialHighlighting();
   }

   private void updateMaterialHighlighting()
   {
      // could only do this when selection changed
      for (Axis3D axis : Axis3D.values)
      {
         if (closestCollisionSelection != null && closestCollisionSelection.isAngular() && closestCollisionSelection.toAxis3D() == axis)
         {
            torusModels[axis.ordinal()].setMaterial(highlightedMaterials[axis.ordinal()]);
         }
         else
         {
            torusModels[axis.ordinal()].setMaterial(normalMaterials[axis.ordinal()]);
         }

         if (closestCollisionSelection != null && closestCollisionSelection.isLinear() && closestCollisionSelection.toAxis3D() == axis)
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
      ImGui.text("Use the right mouse button to manipulate the widget.");

      if (ImGui.button("Reset"))
      {
         transformToParent.setToZero();
      }

      ImGui.checkbox("Resize based on camera distance", resizeAutomatically);
      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Torus radius"), torusRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Torus camera size"), torusCameraSize.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Torus tube radius ratio"), torusTubeRadiusRatio.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow length ratio"), arrowLengthRatio.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow head body length ratio"), arrowHeadBodyLengthRatio.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow head body radius ratio"), arrowHeadBodyRadiusRatio.getData(), 0.001f, 0.0f, 3.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow spacing factor"), arrowSpacingFactor.getData(), 0.001f, 0.0f, 1000.0f);
      ImGui.popItemWidth();

      if (proportionsChanged)
         recreateGraphics();

      updateTransforms();
   }

   private void recreateGraphics()
   {
      if (lastDistanceToCamera > 0.0)
         torusRadius.set(torusCameraSize.get() * (float) lastDistanceToCamera);
      else
         torusRadius.set(torusCameraSize.get());
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

   public Pose3DReadOnly getPose()
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
      GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

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
      return GDXMeshDataInterpreter.interpretMeshData(tetrahedronRingMeshDataHolder(ringRadius, tetrahedronSize, numberOfTetrahedrons));
   }

   public static MeshDataHolder tetrahedronRingMeshDataHolder(double ringRadius, double tetrahedronSize, int numberOfTetrahedrons)
   {
      GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

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

   public void setResizeAutomatically(boolean resizeAutomatically)
   {
      this.resizeAutomatically.set(resizeAutomatically);
   }
}
