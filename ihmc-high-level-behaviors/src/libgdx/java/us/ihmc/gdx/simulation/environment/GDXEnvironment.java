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
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
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
import us.ihmc.gdx.simulation.GDXDoorSimulator;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.objects.*;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.GDXPose3DWidget;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.*;

public class GDXEnvironment implements RenderableProvider
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironment.class, "Environment");
   private final ArrayList<GDXEnvironmentObject> objects = new ArrayList<>();
   private GDXEnvironmentObject selectedObject;
   private GDXEnvironmentObject intersectedObject;
   private ImBoolean show3DWidgetTuner = new ImBoolean(false);
   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget(getClass().getSimpleName());
   private boolean placing = false;
   private boolean loadedFilesOnce = false;
   private Path selectedEnvironmentFile = null;
   private final TreeSet<Path> environmentFiles = new TreeSet<>(Comparator.comparing(path -> path.getFileName().toString()));
   private final ImString saveString = new ImString("", 100);
   private final Point3D tempTranslation = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Point3D tempIntersection = new Point3D();
   private GDXDoorSimulator doorSimulator;

   public GDXEnvironment()
   {

   }

   public GDXEnvironment(ROS2SyncedRobotModel syncedRobot, CommunicationHelper helper)
   {
      doorSimulator = new GDXDoorSimulator(syncedRobot, helper);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      baseUI.getSceneManager().addRenderableProvider(this);

      pose3DWidget.create();
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
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
            selectedObject.set(pickPoint);
            pose3DWidget.getTransform().set(selectedObject.getObjectTransform());

            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               placing = false;
            }
         }
         else
         {
            pose3DWidget.process3DViewInput(viewInput);
            selectedObject.set(pose3DWidget.getTransform());

            intersectedObject = calculatePickedObject(viewInput.getPickRayInWorld());
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               if (intersectedObject != selectedObject)
               {
                  selectedObject = intersectedObject;
                  if (selectedObject != null)
                  {
                     pose3DWidget.getTransform().set(selectedObject.getObjectTransform());
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
               pose3DWidget.getTransform().set(selectedObject.getObjectTransform());
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

      GDXEnvironmentObject objectToPlace = null;
      if (!placing)
      {
//         if (ImGui.button("Place Small Cinder Block Roughed"))
//            objectToPlace = new GDXSmallCinderBlockRoughed();
         if (ImGui.button("Place Medium Cinder Block Roughed"))
            objectToPlace = new GDXMediumCinderBlockRoughed();
//         if (ImGui.button("Place Large Cinder Block Roughed"))
//            objectToPlace = new GDXLargeCinderBlockRoughed();
         if (ImGui.button("Place Lab Floor"))
            objectToPlace = new GDXLabFloorObject();
         if (ImGui.button("Place Pallet"))
            objectToPlace = new GDXPalletObject();
         if (ImGui.button("Place Stairs"))
            objectToPlace = new GDXStairsObject();
         if (ImGui.button("Place Door Frame"))
            objectToPlace = new GDXDoorFrameObject();
         if (ImGui.button("Place Push Door Only"))
         {
            objectToPlace = new GDXPushHandleRightDoorObject();
            if (doorSimulator != null)
               doorSimulator.setDoor((GDXPushHandleRightDoorObject) objectToPlace);
         }
//         if (ImGui.button("Place Door Frame"))
//            objectToPlace = new GDXDoorFrameObject();
      }
      if (objectToPlace != null)
      {
         objects.add(objectToPlace);
         selectedObject = objectToPlace;
         placing = true;
      }

      if (selectedObject != null && ImGui.button("Delete selected"))
      {
         objects.remove(selectedObject);
         selectedObject = null;
         intersectedObject = null;
      }

      ImGui.text("Environments:");
      if (!loadedFilesOnce && selectedEnvironmentFile != null)
      {
         loadedFilesOnce = true;
         loadEnvironment(selectedEnvironmentFile);
      }
      boolean reindexClicked = ImGui.button(ImGuiTools.uniqueLabel(this, "Reindex scripts"));
      if (!loadedFilesOnce || reindexClicked)
      {
         loadedFilesOnce = true;
         reindexScripts();
      }
      String fileNameToSave = null;
      for (Path environmentFile : environmentFiles)
      {
         if (ImGui.radioButton(environmentFile.getFileName().toString(), selectedEnvironmentFile != null && selectedEnvironmentFile.equals(environmentFile)))
         {
            loadEnvironment(environmentFile);
         }
         if (selectedEnvironmentFile != null && selectedEnvironmentFile.equals(environmentFile))
         {
            ImGui.sameLine();
            if (ImGui.button("Save"))
            {
               fileNameToSave = environmentFile.getFileName().toString();
            }
         }
      }
      int flags = ImGuiInputTextFlags.None;
      flags += ImGuiInputTextFlags.CallbackResize;
      ImGui.inputText("###saveText", saveString, flags);
      ImGui.sameLine();
      if (ImGui.button("Save as"))
      {
         fileNameToSave = saveString.get();
      }
      if (fileNameToSave != null)
      {
         JSONFileTools.saveToClasspath("ihmc-open-robotics-software",
                                       "ihmc-high-level-behaviors/src/libgdx/resources",
                                       "environments/" + fileNameToSave,
         rootNode ->
         {
            ArrayNode objectsArrayNode = rootNode.putArray("objects");
            for (GDXEnvironmentObject object : this.objects)
            {
               ObjectNode objectNode = objectsArrayNode.addObject();
               objectNode.put("type", object.getClass().getSimpleName());
               tempTransform.set(object.getObjectTransform());
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
         reindexScripts();
      }

      ImGui.checkbox("Show 3D Widget Tuner", show3DWidgetTuner);
      if (show3DWidgetTuner.get())
         pose3DWidget.renderImGuiTuner();

      if (doorSimulator != null)
         doorSimulator.renderImGuiWidgets();

      ImGui.end();
   }

   private void loadEnvironment(Path environmentFile)
   {
      selectedEnvironmentFile = environmentFile;
      objects.clear();
      selectedObject = null;
      intersectedObject = null;
      if (doorSimulator != null)
         doorSimulator.setDoor(null);

      if (loadedFilesOnce)
      {
         JSONFileTools.loadFromWorkspace("ihmc-open-robotics-software",
                                         "ihmc-high-level-behaviors/src/libgdx/resources",
                                         "environments/" + environmentFile.getFileName().toString(),
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

               if (object instanceof GDXPushHandleRightDoorObject && doorSimulator != null)
               {
                  doorSimulator.setDoor((GDXPushHandleRightDoorObject) object);
               }
            }
         });
      }
   }

   public void loadEnvironment(String environmentFileName)
   {
      reindexScripts();
      Optional<Path> match = environmentFiles.stream().filter(path -> path.getFileName().toString().equals(environmentFileName)).findFirst();
      if (match.isPresent())
      {
         loadEnvironment(match.get());
      }
      else
      {
         LogTools.error("Could not find environment file: {}", environmentFileName);
      }
   }

   private void reindexScripts()
   {
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

   public void destroy()
   {
      if (doorSimulator != null)
         doorSimulator.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public GDXDoorSimulator getDoorSimulator()
   {
      return doorSimulator;
   }
}
