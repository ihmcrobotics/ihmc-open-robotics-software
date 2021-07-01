package us.ihmc.gdx.ui.gizmo;


import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;

public class GDXFootstepPlannerGoalGizmo implements RenderableProvider
{
   private final double QUARTER_TURN = Math.PI / 2.0;
   private final ImFloat discOuterRadius = new ImFloat(0.426f);
   private final ImFloat discInnerRadius = new ImFloat(0.290f);
   private final ImFloat discThickness = new ImFloat(0.014f);
   private final ImFloat arrowWidth = new ImFloat(0.257f);
   private final ImFloat arrowHeight = new ImFloat(0.137f);
   private final ImFloat arrowSpacing = new ImFloat(0.079f);
   private final Pose3D pose = new Pose3D();
   /** The main, source, true, base transform that this thing represents. */
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection;
   private boolean dragging = false;
   private final String imGuiWindowName;
   private FocusBasedGDXCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double lastDistanceToCamera = -1.0;
   private DynamicGDXModel discModel = new DynamicGDXModel();
   private DynamicGDXModel positiveXArrowModel = new DynamicGDXModel();
   private DynamicGDXModel positiveYArrowModel = new DynamicGDXModel();
   private DynamicGDXModel negativeXArrowModel = new DynamicGDXModel();
   private DynamicGDXModel negativeYArrowModel = new DynamicGDXModel();
   private Material normalDiscMaterial;
   private Material normalArrowMaterial;
   private Material highlightedDiscMaterial;
   private Material highlightedArrowMaterial;

   public GDXFootstepPlannerGoalGizmo(String name)
   {
      imGuiWindowName = ImGuiTools.uniqueLabel("3D Widget (" + name + ")");
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      this.camera3D = camera3D;

      normalDiscMaterial = new Material();
      normalArrowMaterial = new Material();
      highlightedDiscMaterial = new Material();
      highlightedArrowMaterial = new Material();
      discModel.setMesh(meshBuilder ->
      {
         meshBuilder.addHollowCylinder(discThickness.get(),
                                       discOuterRadius.get(),
                                       discInnerRadius.get(),
                                       new Point3D(0.0, 0.0, -discThickness.get() / 2.0),
                                       Color.LIGHT_GRAY);
      });
      positiveXArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(discOuterRadius.get() + arrowSpacing.get(), 0.0, -discThickness.get() / 2.0),
                                                 new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN),
                                                 Color.YELLOW);
      });
      positiveYArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(0.0, discOuterRadius.get() + arrowSpacing.get(), -discThickness.get() / 2.0),
                                                 new YawPitchRoll(0.0, 0.0, -QUARTER_TURN),
                                                 Color.YELLOW);
      });
      negativeXArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(-discOuterRadius.get() - arrowSpacing.get(), 0.0, -discThickness.get() / 2.0),
                                                 new YawPitchRoll(QUARTER_TURN, 0.0, -QUARTER_TURN),
                                                 Color.YELLOW);
      });
      negativeYArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(0.0, -discOuterRadius.get() - arrowSpacing.get(), -discThickness.get() / 2.0),
                                                 new YawPitchRoll(0.0, 0.0, QUARTER_TURN),
                                                 Color.YELLOW);
      });


      recreateGraphics();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      updateFromSourceTransform();

      boolean rightMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Right);
      boolean isWindowHovered = ImGui.isWindowHovered();

      if (!rightMouseDown)
      {
         dragging = false;
      }
      if (isWindowHovered && !dragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (rightMouseDown)
         {
            dragging = true;
         }
      }
      if (dragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

      }

      // after things have been modified, update the derivative stuff
      updateFromSourceTransform();

      GDXTools.toEuclid(camera3D.position, cameraPosition);
      double distanceToCamera = cameraPosition.distance(pose.getPosition());
      if (lastDistanceToCamera != distanceToCamera)
      {
         lastDistanceToCamera = distanceToCamera;
         recreateGraphics();
         updateFromSourceTransform();
      }
   }

   private void updateFromSourceTransform()
   {
      pose.set(transform);
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      closestCollisionSelection = null;
      double closestCollisionDistance = Double.POSITIVE_INFINITY;

      updateMaterialHighlighting();
   }

   private void updateMaterialHighlighting()
   {
      discModel.setMaterial(normalDiscMaterial);
      positiveXArrowModel.setMaterial(normalArrowMaterial);
      positiveYArrowModel.setMaterial(normalArrowMaterial);
      negativeXArrowModel.setMaterial(normalArrowMaterial);
      negativeYArrowModel.setMaterial(normalArrowMaterial);
   }

   public void renderImGuiTuner()
   {
      ImGui.begin(imGuiWindowName);
      ImGui.text("Use the right mouse button to manipulate the widget.");

      if (ImGui.button("Reset"))
      {
         transform.setToZero();
      }

      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc outer radius"), discOuterRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc inner radius"), discInnerRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Disc thickness"), discThickness.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow width"), arrowWidth.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow height"), arrowHeight.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow spacing"), arrowSpacing.getData(), 0.001f, 0.0f, 1000.0f);
      ImGui.popItemWidth();

      if (proportionsChanged)
         recreateGraphics();

      ImGui.end();

      updateFromSourceTransform();
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
      positiveXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      positiveYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      negativeXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      negativeYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
   }

   public Pose3DReadOnly getPose()
   {
      return pose;
   }

   // TODO: Make this transform the ground truth and give the pose as needed only
   public RigidBodyTransform getTransform()
   {
      return transform;
   }
}
