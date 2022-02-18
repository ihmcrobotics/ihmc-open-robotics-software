package us.ihmc.gdx.ui.gizmo;


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
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXFootstepPlannerGoalGizmo implements RenderableProvider
{
   public static final Color LIGHT_GRAY = new Color().fromHsv(0.0f, 0.0f, 0.836f);
   public static final Color LIGHTER_GRAY = new Color().fromHsv(0.0f, 0.0f, 0.95f);
   public static final Color YELLOW_HIGHLIGHT = new Color().fromHsv(61.5f, 0.783f, 0.892f);
   public static final Color DISC_NORMAL_COLOR = LIGHT_GRAY;
   public static final Color DISC_HIGHLIGHTED_COLOR = LIGHTER_GRAY;
   public static final Color ARROW_NORMAL_COLOR = LIGHT_GRAY;
   public static final Color ARROW_HIGHLIGHTED_COLOR = LIGHTER_GRAY;
   static
   {
      DISC_NORMAL_COLOR.a = 0.4f;
      ARROW_NORMAL_COLOR.a = 0.4f;
      DISC_HIGHLIGHTED_COLOR.a = 0.9f;
      ARROW_HIGHLIGHTED_COLOR.a = 0.9f;
   }
   private final double QUARTER_TURN = Math.PI / 2.0;
   private final ImFloat discOuterRadius = new ImFloat(0.426f);
   private final ImFloat discInnerRadius = new ImFloat(0.290f);
   private final ImFloat discThickness = new ImFloat(0.014f);
   private final ImFloat arrowWidth = new ImFloat(0.257f);
   private final ImFloat arrowHeight = new ImFloat(0.137f);
   private final ImFloat arrowSpacing = new ImFloat(0.079f);
   private Material normalDiscMaterial;
   private Material normalArrowMaterial;
   private Material highlightedDiscMaterial;
   private Material highlightedArrowMaterial;
   private DynamicGDXModel discModel = new DynamicGDXModel();
   private DynamicGDXModel positiveXArrowModel = new DynamicGDXModel();
   private DynamicGDXModel positiveYArrowModel = new DynamicGDXModel();
   private DynamicGDXModel negativeXArrowModel = new DynamicGDXModel();
   private DynamicGDXModel negativeYArrowModel = new DynamicGDXModel();
   private final Point3D closestCollision = new Point3D();
   private int closestCollisionSelection = -1;
   private final HollowCylinderRayIntersection hollowCylinderIntersection = new HollowCylinderRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveXArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection positiveYArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection negativeXArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final DiscreteIsoscelesTriangularPrismRayIntersection negativeYArrowIntersection = new DiscreteIsoscelesTriangularPrismRayIntersection();
   private final Pose3D pose = new Pose3D();
   /** The main, source, true, base transform that this thing represents. */
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                        transform);
   private FocusBasedGDXCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
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
   private boolean isBeingDragged;

   public GDXFootstepPlannerGoalGizmo()
   {
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      this.camera3D = camera3D;

      normalDiscMaterial = new Material();
      normalDiscMaterial.set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
      normalDiscMaterial.set(new BlendingAttribute(true, DISC_NORMAL_COLOR.a));
      normalArrowMaterial = new Material();
      normalArrowMaterial.set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
      normalArrowMaterial.set(new BlendingAttribute(true, ARROW_NORMAL_COLOR.a));
      highlightedDiscMaterial = new Material();
      highlightedDiscMaterial.set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
      highlightedDiscMaterial.set(new BlendingAttribute(true, DISC_HIGHLIGHTED_COLOR.a));
      highlightedArrowMaterial = new Material();
      highlightedArrowMaterial.set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
      highlightedArrowMaterial.set(new BlendingAttribute(true, ARROW_HIGHLIGHTED_COLOR.a));
      discModel.setMesh(meshBuilder ->
      {
         meshBuilder.addHollowCylinder(discThickness.get(),
                                       discOuterRadius.get(),
                                       discInnerRadius.get(),
                                       new Point3D(0.0, 0.0, -discThickness.get() / 2.0),
                                       DISC_NORMAL_COLOR);
      });
      positiveXArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(discOuterRadius.get() + arrowSpacing.get(), 0.0, 0.0),
                                                 new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN),
                                                 ARROW_NORMAL_COLOR);
      });
      positiveYArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(0.0, discOuterRadius.get() + arrowSpacing.get(), 0.0),
                                                 new YawPitchRoll(0.0, 0.0, -QUARTER_TURN),
                                                 ARROW_NORMAL_COLOR);
      });
      negativeXArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(-discOuterRadius.get() - arrowSpacing.get(), 0.0, 0.0),
                                                 new YawPitchRoll(QUARTER_TURN, 0.0, -QUARTER_TURN),
                                                 ARROW_NORMAL_COLOR);
      });
      negativeYArrowModel.setMesh(meshBuilder ->
      {
         meshBuilder.addIsoscelesTriangularPrism(arrowWidth.get(),
                                                 arrowHeight.get(),
                                                 discThickness.get(),
                                                 new Point3D(0.0, -discOuterRadius.get() - arrowSpacing.get(), 0.0),
                                                 new YawPitchRoll(0.0, 0.0, QUARTER_TURN),
                                                 ARROW_NORMAL_COLOR);
      });

      recreateGraphics();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      updateFromSourceTransform();

      boolean rightMouseDragging = input.isDragging(ImGuiMouseButton.Right);
      boolean middleMouseDragging = input.isDragging(ImGuiMouseButton.Middle);
      boolean middleMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Middle);
      boolean isWindowHovered = ImGui.isWindowHovered();
      isBeingDragged = false;

      if (isWindowHovered && !rightMouseDragging && !middleMouseDragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (middleMouseDown && closestCollisionSelection > -1)
         {
            clockFaceDragAlgorithm.reset();
         }
      }
      if (rightMouseDragging || middleMouseDragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

         if (closestCollisionSelection == 0)
         {
            isBeingDragged = true;
            if (rightMouseDragging)
            {
               Vector3DReadOnly planarMotion = planeDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z);
               transform.getTranslation().add(planarMotion);
               closestCollision.add(planarMotion);
            }
            else // middleMouseDragging
            {
               if (clockFaceDragAlgorithm.calculate(pickRay, closestCollision, Axis3D.Z, transform))
               {
                  clockFaceDragAlgorithm.getMotion().transform(transform.getRotation());
               }
            }
         }
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
      GDXTools.toGDX(transform, discModel.getOrCreateModelInstance().transform);
      GDXTools.toGDX(transform, positiveXArrowModel.getOrCreateModelInstance().transform);
      GDXTools.toGDX(transform, positiveYArrowModel.getOrCreateModelInstance().transform);
      GDXTools.toGDX(transform, negativeXArrowModel.getOrCreateModelInstance().transform);
      GDXTools.toGDX(transform, negativeYArrowModel.getOrCreateModelInstance().transform);
      referenceFrame.update();
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      hollowCylinderIntersects = false;
      positiveXArrowIntersects = false;
      positiveYArrowIntersects = false;
      negativeXArrowIntersects = false;
      negativeYArrowIntersects = false;
      closestCollisionSelection = -1;
      double closestCollisionDistance = Double.POSITIVE_INFINITY;

      hollowCylinderIntersection.setup(discThickness.get(), discOuterRadius.get(), discInnerRadius.get(), 0.0, transform);
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
         positiveXArrowIntersection.setup(arrowWidth.get(),
                                          arrowHeight.get(),
                                          discThickness.get(),
                                          new Point3D(discOuterRadius.get() + arrowSpacing.get(), 0.0, 0.0),
                                          new YawPitchRoll(-QUARTER_TURN, 0.0, -QUARTER_TURN),
                                          transform);
         distance = positiveXArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            positiveXArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 1;
            closestCollision.set(positiveXArrowIntersection.getClosestIntersection());
         }
         positiveYArrowIntersection.setup(arrowWidth.get(),
                                          arrowHeight.get(),
                                          discThickness.get(),
                                          new Point3D(0.0, discOuterRadius.get() + arrowSpacing.get(), 0.0),
                                          new YawPitchRoll(0.0, 0.0, -QUARTER_TURN),
                                          transform);
         distance = positiveYArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            positiveYArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 2;
            closestCollision.set(positiveYArrowIntersection.getClosestIntersection());
         }
         negativeXArrowIntersection.setup(arrowWidth.get(),
                                          arrowHeight.get(),
                                          discThickness.get(),
                                          new Point3D(-discOuterRadius.get() - arrowSpacing.get(), 0.0, 0.0),
                                          new YawPitchRoll(QUARTER_TURN, 0.0, -QUARTER_TURN),
                                          transform);
         distance = negativeXArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            negativeXArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 3;
            closestCollision.set(negativeXArrowIntersection.getClosestIntersection());
         }
         negativeYArrowIntersection.setup(arrowWidth.get(),
                                          arrowHeight.get(),
                                          discThickness.get(),
                                          new Point3D(0.0, -discOuterRadius.get() - arrowSpacing.get(), 0.0),
                                          new YawPitchRoll(0.0, 0.0, QUARTER_TURN),
                                          transform);
         distance = negativeYArrowIntersection.intersect(pickRay, 100);
         if (!Double.isNaN(distance) && distance < closestCollisionDistance)
         {
            negativeYArrowIntersects = true;
            closestCollisionDistance = distance;
            closestCollisionSelection = 4;
            closestCollision.set(negativeYArrowIntersection.getClosestIntersection());
         }
      }

      updateMaterialHighlighting();
   }

   private void updateMaterialHighlighting()
   {
      discModel.setMaterial(highlightingEnabled && closestCollisionSelection == 0 ? highlightedDiscMaterial : normalDiscMaterial);
      positiveXArrowModel.setMaterial(highlightingEnabled && closestCollisionSelection == 1 ? highlightedArrowMaterial : normalArrowMaterial);
      positiveYArrowModel.setMaterial(highlightingEnabled && closestCollisionSelection == 2 ? highlightedArrowMaterial : normalArrowMaterial);
      negativeXArrowModel.setMaterial(highlightingEnabled && closestCollisionSelection == 3 ? highlightedArrowMaterial : normalArrowMaterial);
      negativeYArrowModel.setMaterial(highlightingEnabled && closestCollisionSelection == 4 ? highlightedArrowMaterial : normalArrowMaterial);
   }

   public ImGuiPanel createTunerPanel(String name)
   {
      return new ImGuiPanel("Footstep Ring Gizmo Tuner (" + name + ")", this::renderImGuiTuner);
   }

   public void renderImGuiTuner()
   {
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
      if (showArrows)
      {
         positiveXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         positiveYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         negativeXArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
         negativeYArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
      }
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

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public boolean getIntersectsAny()
   {
      return hollowCylinderIntersects || positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects;
   }

   public boolean getIntersectsAnyArrow()
   {
      return positiveXArrowIntersects || positiveYArrowIntersects || negativeXArrowIntersects || negativeYArrowIntersects;
   }

   public boolean getHollowCylinderIntersects()
   {
      return hollowCylinderIntersects;
   }

   public boolean getPositiveXArrowIntersects()
   {
      return positiveXArrowIntersects;
   }

   public boolean getPositiveYArrowIntersects()
   {
      return positiveYArrowIntersects;
   }

   public boolean getNegativeXArrowIntersects()
   {
      return negativeXArrowIntersects;
   }

   public boolean getNegativeYArrowIntersects()
   {
      return negativeYArrowIntersects;
   }

   public void setShowArrows(boolean showArrows)
   {
      this.showArrows = showArrows;
   }

   public void setHighlightingEnabled(boolean highlightingEnabled)
   {
      this.highlightingEnabled = highlightingEnabled;
   }

   public boolean isBeingDragged()
   {
      return isBeingDragged;
   }
}
