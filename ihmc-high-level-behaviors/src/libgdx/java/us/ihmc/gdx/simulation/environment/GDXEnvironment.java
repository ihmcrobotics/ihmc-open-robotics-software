package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXPose3DWidget;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class GDXEnvironment implements RenderableProvider
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironment.class, "Environment");
   private GDXImGuiBasedUI baseUI;
   private final ArrayList<GDXEnvironmentObject> objects = new ArrayList<>();
   private GDXEnvironmentObject selectedObject;
   private GDXEnvironmentObject intersectedObject;
   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget();
   private boolean placing = false;
   private boolean loadedFilesOnce = false;
   private int selectedEnvironmentFile = -1;
   private final ArrayList<Path> environmentFiles = new ArrayList<>();
   private final ImString saveString = new ImString("", 100);
   private final Point3D tempTranslation = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
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

      ImGui.text("Environments:");
      boolean reindexClicked = imgui.internal.ImGui.button(ImGuiTools.uniqueLabel(this, "Reindex scripts"));
      if (!loadedFilesOnce || reindexClicked)
      {
         loadedFilesOnce = true;
         Path scriptsPath = WorkspacePathTools.findPathToResource("ihmc-open-robotics-software",
                                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                                  "environments");
         environmentFiles.clear();
         PathTools.walkFlat(scriptsPath, (path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.FILE)
            {
               environmentFiles.add(path);
            }
            return FileVisitResult.CONTINUE;
         });
      }
      for (int i = 0; i < environmentFiles.size(); i++)
      {
         if (ImGui.radioButton(environmentFiles.get(i).getFileName().toString(), selectedEnvironmentFile == i))
         {
            selectedEnvironmentFile = i;
            objects.clear();
            selectedObject = null;
            intersectedObject = null;

            JSONFileTools.loadFromWorkspace("ihmc-open-robotics-software",
                                            "ihmc-high-level-behaviors/src/libgdx/resources",
                                            "environments/" + environmentFiles.get(i).getFileName().toString(),
            node ->
            {
               for (Iterator<JsonNode> it = node.withArray("objects").elements(); it.hasNext(); )
               {
                  JsonNode objectNode = it.next();
                  GDXEnvironmentObject object = GDXEnvironmentObject.loadByName(objectNode.get("type").asText());
                  tempTranslation.setX(objectNode.get("x").asDouble());
                  tempTranslation.setY(objectNode.get("y").asDouble());
                  tempTranslation.setZ(objectNode.get("z").asDouble());
                  tempOrientation.set(objectNode.get("qx").asDouble(),
                                      objectNode.get("qy").asDouble(),
                                      objectNode.get("qz").asDouble(),
                                      objectNode.get("qs").asDouble());
                  tempTransform.set(tempOrientation, tempTranslation);
                  object.set(tempTransform);
                  objects.add(object);
               }
            });
         }
      }
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText("###saveText", saveString, flags);
      ImGui.sameLine();
      if (ImGui.button("Save as new"))
      {
         JSONFileTools.saveToClasspath("ihmc-open-robotics-software",
                                       "ihmc-high-level-behaviors/src/libgdx/resources",
                                       "environments/" + saveString.get(),
         rootNode ->
         {
            ArrayNode objectsArrayNode = rootNode.putArray("objects");
            for (GDXEnvironmentObject object : this.objects)
            {
               ObjectNode objectNode = objectsArrayNode.addObject();
               objectNode.put("type", object.getClass().getSimpleName());
               GDXTools.toEuclid(object.getRealisticModelInstance().transform, tempTransform);
               tempTranslation.set(tempTransform.getTranslation());
               tempOrientation.set(tempTransform.getRotation());
               objectNode.put("x", tempTranslation.getX());
               objectNode.put("y", tempTranslation.getY());
               objectNode.put("z", tempTranslation.getZ());
               objectNode.put("qx", tempOrientation.getX());
               objectNode.put("qy", tempOrientation.getY());
               objectNode.put("qz", tempOrientation.getZ());
               objectNode.put("qs", tempOrientation.getS());
            }
         });
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
