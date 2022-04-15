package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.object.GDXSimpleObject;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;

import java.util.ArrayList;

public class GDXBuildingConstructor extends ImGuiPanel
{

   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironmentBuilder.class, "Constructor");
   private final ArrayList<GDXSimpleObject> virtualObjects = new ArrayList<>();
   private GDXSimpleObject selectedObject;
   private GDXSimpleObject intersectedObject;
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final GDXPose3DGizmo pose3DGizmo = new GDXPose3DGizmo();
   private final ImGuiPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());

   private final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
   private final GDX3DSceneManager sceneManager;
   private boolean constructionIsInProgress = false;
   private final Point3D tempIntersection = new Point3D();

   public GDXBuildingConstructor(GDX3DSceneManager sceneManager)
   {
      super(WINDOW_NAME);
      this.sceneManager = sceneManager;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      sceneManager.addRenderableProvider(this::getRenderables, GDXSceneLevel.VIRTUAL);
      pose3DGizmo.create(sceneManager.getCamera3D());
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXSimpleObject model : virtualObjects)
      {
         model.getRealRenderables(renderables, pool);
      }
      if (selectedObject != null)
      {
         pose3DGizmo.getRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionMeshRenderables(renderables, pool);
      }
   }
   public void process3DViewInput(ImGui3DViewInput viewInput)
   {
      if (selectedObject != null)
      {
         if (constructionIsInProgress)
         {
            Line3DReadOnly pickRay = viewInput.getPickRayInWorld();
            Point3D pickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                        Axis3D.Z,
                                                                                        pickRay.getPoint(),
                                                                                        pickRay.getDirection());
            selectedObject.setPositionInWorld(pickPoint);
            pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());

            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               constructionIsInProgress = false;
            }
         }
         else
         {
            pose3DGizmo.process3DViewInput(viewInput);
            selectedObject.setTransformToWorld(pose3DGizmo.getTransformToParent());

            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               if (intersectedObject != selectedObject)
               {
                  updateObjectSelected(selectedObject, intersectedObject);
                  if (selectedObject != null)
                  {
                     pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
                  }
               }
            }
         }
      }
      else
      {
         constructionIsInProgress = false;
         if (viewInput.isWindowHovered())
         {
            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());

            if (intersectedObject != null && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               updateObjectSelected(selectedObject, intersectedObject);
               pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
            }
         }
      }
   }

   private GDXSimpleObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      GDXSimpleObject closestObject = null;
      for (GDXSimpleObject object : virtualObjects)
      {
         boolean intersects = object.intersect(pickRay, tempIntersection);
         double distance = tempIntersection.distance(pickRay.getPoint());
         if (intersects && (closestObject == null || distance < closestDistance))
         {
            closestObject = object;
            closestDistance = distance;

         }
      }
      return closestObject;
   }

   public void renderImGuiWidgets()
   {
      ImGui.separator();
      if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
      {
         sceneManager.getSceneBasics().setAmbientLight(ambientLightAmount.get());
      }
      ImGui.separator();
      if (!constructionIsInProgress)
      {
         if (ImGui.button("Create Building"))
         {
            GDXSimpleObject objectToPlace = new GDXSimpleObject("Building");

            Model objectModel = GDXModelPrimitives.createBox(1.0f, 1.0f, 1.0f, Color.CHARTREUSE).model;
            Box3D collisionBox = new Box3D(1.0f, 1.0f, 1.0f);
            objectToPlace.setRealisticModel(objectModel);
            objectToPlace.setCollisionModel(objectModel);
            objectToPlace.setCollisionGeometryObject(collisionBox);

            virtualObjects.add(objectToPlace);
            updateObjectSelected(selectedObject, objectToPlace);
            constructionIsInProgress = true;
         }

         ImGui.separator();
      }
      if (selectedObject != null && (ImGui.button("Delete selected") || ImGui.isKeyReleased(ImGuiTools.getDeleteKey())))
      {
         virtualObjects.remove(selectedObject);
         resetSelection();
      }

      ImGui.checkbox("Show 3D Widget Tuner", poseGizmoTunerPanel.getIsShowing());
   }

   public void updateObjectSelected(GDXSimpleObject from, GDXSimpleObject to)
   {
      if (from != to)
      {
         selectedObject = to;
      }
   }

   public void resetSelection()
   {
      updateObjectSelected(selectedObject, null);
      intersectedObject = null;
   }
}
