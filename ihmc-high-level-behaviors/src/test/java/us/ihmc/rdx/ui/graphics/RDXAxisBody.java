package us.ihmc.rdx.ui.graphics;

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
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.DiscreteTorusRayIntersection;
import us.ihmc.rdx.ui.gizmo.DynamicLibGDXModel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.visualizers.RDXLineMeshModel;

import java.util.ArrayList;

public class RDXAxisBody implements RenderableProvider
{
   public static final float DEFAULT_TORUS_RADIUS = 1.5f;
   private RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private FramePoint3D pointA = new FramePoint3D();
   private FramePoint3D pointB = new FramePoint3D();
   private double lineLength = 2.0;
   private RDXLineMeshModel lineMeshModel = new RDXLineMeshModel(0.01f, Color.YELLOW);

   private final DynamicLibGDXModel torusModel = new DynamicLibGDXModel();

   // in world
   private ArrayList<Point3DReadOnly> linePts = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   // -----------> for torus (circle)
   private final ImFloat torusRadius = new ImFloat(1.0f);
   private final ImFloat torusCameraSize = new ImFloat(0.067f);
   private final ImFloat torusTubeRadiusRatio = new ImFloat(0.024f);
   private final RigidBodyTransform axisTransformToWorld = new RigidBodyTransform();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private boolean collisionWithTorus = false;
   private double closestCollisionDistance;
   private final Point3D closestCollision = new Point3D();
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean leftButtonDown = false;
   private boolean rightButtonDown = false;
   // < ----------------

   public RDXAxisBody(RDXBaseUI baseUI)
   {
      poseGizmo.create(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
      baseUI.getPrimaryScene().addRenderableProvider(poseGizmo);

      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);

      pointA.set(poseGizmo.getPose().getPosition());
      pointA.addZ(lineLength / 2.0);

      pointB.set(poseGizmo.getPose().getPosition());
      pointB.addZ(-lineLength / 2.0);
      linePts.add(pointA);
      linePts.add(pointB);
      lineMeshModel.generateMeshForMatchLines(linePts);
      lineMeshModel.update();

      int resolution = 30;
      torusModel.setMesh(
        meshBuilder -> meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), resolution, Color.GOLD)
      );
      Material material = new Material();
      material.set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      material.set(new BlendingAttribute(true, 1.0f));
      torusModel.setMaterial(material);

      recreateTorusGraphics();
   }

   private void updateLineFromGizmo()
   {
      pointA.setToZero();
      pointA.addZ(lineLength / 2.0);
      pointA.applyTransform(poseGizmo.getTransformToParent());

      pointB.setToZero();
      pointB.addZ(-lineLength / 2.0);
      pointB.applyTransform(poseGizmo.getTransformToParent());

      linePts.set(0, pointA);
      pointA.changeFrame(poseGizmo.getGizmoFrame());

      linePts.set(1, pointB);
      pointB.changeFrame(poseGizmo.getGizmoFrame());

      if (lineMeshModel.getModelInstance() != null)
      {
         LibGDXTools.toLibGDX(poseGizmo.getPose(), tempTransform, lineMeshModel.getModelInstance().transform);
         lineMeshModel.update();
      }
   }

   public void update()
   {
      poseGizmo.update();
      updateLineFromGizmo();
      FramePose3D temp = new FramePose3D(poseGizmo.getPose());
      temp.get(axisTransformToWorld);
      recreateTorusGraphics();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (lineMeshModel != null)
      {
         lineMeshModel.getRenderables(renderables, pool);
      }
      if (poseGizmo != null)
      {
         poseGizmo.getRenderables(renderables, pool);
      }
      torusModel.getOrCreateModelInstance().getRenderables(renderables, pool);
   }

   private void recreateTorusGraphics()
   {
      torusModel.invalidateMesh();
      LibGDXTools.toLibGDX(axisTransformToWorld, torusModel.getOrCreateModelInstance().transform);
   }

   public RDXPose3DGizmo getPoseGizmo()
   {
      return poseGizmo;
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      leftButtonDown = ImGui.isMouseDown(ImGuiMouseButton.Left);
      rightButtonDown = ImGui.isMouseDown(ImGuiMouseButton.Right);

      boolean pickedTorus = pickResult == input.getClosestPick() && (leftButtonDown || rightButtonDown);

      if (isWindowHovered && pickedTorus)
      {
         if (leftButtonDown)
            torusRadius.set(torusRadius.get() + 0.01f);
         else
            torusRadius.set(torusRadius.get() - 0.01f);
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      boolean isWindowHovered = ImGui.isWindowHovered();
      ImGuiMouseDragData manipulationDragData = input.getMouseDragData(ImGuiMouseButton.Left);

      // Here we are trying to avoid unnecessary computation in collision calculation by filtering out
      // some common scenarios where we don't need to calculate the pick, which can be expensive
      if (isWindowHovered && (!manipulationDragData.isDragging() || manipulationDragData.getDragJustStarted()))
      {
         // This part is happening when the user could presumably start a drag on this gizmo at any time
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineMouseCollisionWithTorus(pickRay);

         if (collisionWithTorus)
         {
            pickResult.setDistanceToCamera(closestCollisionDistance);
            input.addPickResult(pickResult);
         }
      }
   }

   private void determineMouseCollisionWithTorus(Line3DReadOnly pickRay)
   {
      collisionWithTorus = false;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      torusIntersection.update(torusRadius.get(), torusTubeRadiusRatio.get() * torusRadius.get(), axisTransformToWorld);
      double distance = torusIntersection.intersect(pickRay, 100);
      if (!Double.isNaN(distance) && distance < closestCollisionDistance)
      {
         closestCollisionDistance = distance;
         collisionWithTorus = true;
         closestCollision.set(torusIntersection.getClosestIntersection());
      }
   }
}
