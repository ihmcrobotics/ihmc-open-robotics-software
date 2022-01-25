package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXDirectionalLightObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXPointLightObject;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

import java.util.ArrayList;
import java.util.function.Consumer;

public class GDXEnvironmentObjectInteraction
{
   private boolean placing = false;
   private GDXEnvironmentObject selectedObject;
   private GDXEnvironmentObject intersectedObject;
   private final GDXPose3DGizmo pose3DGizmo = new GDXPose3DGizmo();
   private final ImGuiPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());
   private final Point3D tempIntersection = new Point3D();
   private GDX3DSceneManager sceneManager;
   private ArrayList<GDXEnvironmentObject> allObjects;
   private ArrayList<GDXEnvironmentObject> lightObjects;
   private Consumer<GDXEnvironmentObject> addObject;
   private Consumer<GDXEnvironmentObject> removeObject;

   public void create(GDX3DSceneManager sceneManager,
                      ArrayList<GDXEnvironmentObject> allObjects,
                      ArrayList<GDXEnvironmentObject> lightObjects,
                      Consumer<GDXEnvironmentObject> addObject,
                      Consumer<GDXEnvironmentObject> removeObject)
   {
      this.sceneManager = sceneManager;
      this.allObjects = allObjects;
      this.lightObjects = lightObjects;
      this.addObject = addObject;
      this.removeObject = removeObject;
      pose3DGizmo.create(sceneManager.getCamera3D());
   }

   public void process3DViewInput(ImGui3DViewInput viewInput)
   {
      if (selectedObject != null)
      {
         if (placing)
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
               placing = false;
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
                  selectedObject = intersectedObject;
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
         if (viewInput.isWindowHovered())
         {
            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());

            if (intersectedObject != null && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               selectedObject = intersectedObject;
               pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
            }
         }
      }
   }

   private GDXEnvironmentObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      GDXEnvironmentObject closestObject = null;
      for (GDXEnvironmentObject object : allObjects)
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

   public void handleObjectPlacementAndRemoval(GDXEnvironmentObject objectToPlace)
   {
      if (objectToPlace != null)
      {
         addObject.accept(objectToPlace);
         selectedObject = objectToPlace;
         placing = true;
      }

      if (selectedObject != null && (ImGui.button("Delete selected") || ImGui.isKeyReleased(ImGuiTools.getDeleteKey())))
      {
         removeObject.accept(selectedObject);
         resetSelection();
      }
   }

   public void renderImGuiWidgets()
   {

   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selectedObject != null)
      {
         selectedObject.getCollisionModelInstance().getRenderables(renderables, pool);
         pose3DGizmo.getRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionModelInstance().getRenderables(renderables, pool);
      }
   }

   public void resetSelection()
   {
      selectedObject = null;
      intersectedObject = null;
   }

   public ImGuiPanel getPoseGizmoTunerPanel()
   {
      return poseGizmoTunerPanel;
   }

   public boolean isPlacing()
   {
      return placing;
   }

   public GDXEnvironmentObject getSelectedObject()
   {
      return selectedObject;
   }

   public GDXEnvironmentObject getIntersectedObject()
   {
      return intersectedObject;
   }
}
