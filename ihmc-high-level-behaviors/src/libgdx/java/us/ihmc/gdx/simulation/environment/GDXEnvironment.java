package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXPose3DWidget;

import java.util.ArrayList;

public class GDXEnvironment implements RenderableProvider
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironment.class, "Environment");
   private GDXImGuiBasedUI baseUI;
   private final ArrayList<GDXEnvironmentObject> objects = new ArrayList<>();
   private GDXEnvironmentObject selectedObject;
   private GDXEnvironmentObject intersectedObject;
   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget();
   private boolean placing = false;

   private final Point3D tempIntersection = new Point3D();

   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
      baseUI.getSceneManager().addRenderableProvider(this);

      pose3DWidget.create(baseUI);
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
   {
      if (selectedObject != null)
      {
         if (placing)
         {
            Line3DReadOnly pickRay = viewInput.getPickRayInWorld(baseUI);
            Point3D pickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                        Axis3D.Z,
                                                                                        pickRay.getPoint(),
                                                                                        pickRay.getDirection());
            selectedObject.set(pickPoint);
            GDXTools.toEuclid(selectedObject.getRealisticModelInstance().transform, pose3DWidget.getTransform());

            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placing = false;
            }
         }
         else
         {
            pose3DWidget.process3DViewInput(viewInput);
            selectedObject.set(pose3DWidget.getTransform());

            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld(baseUI));
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               if (intersectedObject != selectedObject)
               {
                  selectedObject = intersectedObject;
                  if (selectedObject != null)
                  {
                     GDXTools.toEuclid(selectedObject.getRealisticModelInstance().transform, pose3DWidget.getTransform());
                  }
               }
            }
         }
      }
      else
      {
         if (viewInput.isWindowHovered())
         {
            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld(baseUI));

            if (intersectedObject != null && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               selectedObject = intersectedObject;
               GDXTools.toEuclid(selectedObject.getRealisticModelInstance().transform, pose3DWidget.getTransform());
            }
         }
      }
   }

   private GDXEnvironmentObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      GDXEnvironmentObject closestObject = null;
      for (GDXEnvironmentObject object : objects)
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

   public void render()
   {
      ImGui.begin(WINDOW_NAME);

      ImGui.text("Selected: " + selectedObject);
      ImGui.text("Intersecting: " + intersectedObject);

      if (ImGui.button("Place Medium Cinder Block Roughed"))
      {
         GDXMediumCinderBlockRoughed mediumCinderBlockRoughed = new GDXMediumCinderBlockRoughed();
         objects.add(mediumCinderBlockRoughed);
         selectedObject = mediumCinderBlockRoughed;
         placing = true;
      }

      if (selectedObject != null && ImGui.button("Delete selected"))
      {
         objects.remove(selectedObject);
         selectedObject = null;
         intersectedObject = null;
      }

      pose3DWidget.render();

      ImGui.end();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXEnvironmentObject object : objects)
      {
         object.getRealisticModelInstance().getRenderables(renderables, pool);
      }

      if (selectedObject != null)
      {
         selectedObject.getCollisionModelInstance().getRenderables(renderables, pool);
         pose3DWidget.getRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionModelInstance().getRenderables(renderables, pool);
      }
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
