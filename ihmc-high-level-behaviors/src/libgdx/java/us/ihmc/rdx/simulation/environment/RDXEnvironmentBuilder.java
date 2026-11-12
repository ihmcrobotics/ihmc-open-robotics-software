package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.isaac.RDXIsaacSimEnvironmentExporter;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectLibrary;
import us.ihmc.rdx.simulation.environment.object.objects.RDXDirectionalLightObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXPointLightObject;
import us.ihmc.rdx.simulation.environment.object.objects.terrain.RDXProceduralGroundObject;
import us.ihmc.rdx.simulation.environment.object.objects.terrain.RDXProceduralTerrainObject;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class RDXEnvironmentBuilder extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final WorkspaceResourceDirectory environmentFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/environments");
   private final ArrayList<RDXEnvironmentObject> allObjects = new ArrayList<>();
   private final ArrayList<RDXEnvironmentObject> lightObjects = new ArrayList<>();
   private final ArrayList<RDXProceduralTerrainObject> proceduralTerrainObjects = new ArrayList<>();
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();
   private final RDX3DPanel panel3D;

   private final TreeSet<String> environmentFileNames = new TreeSet<>(RDXEnvironmentBuilder::compareFileNames);
   /** File name to the shared name of the files it is listed with, for files listed in a group */
   private final Map<String, String> environmentGroups = new HashMap<>();
   /** File name to what tells it apart within its group: "3", "Pull Door" */
   private final Map<String, String> environmentVariantNames = new HashMap<>();
   /** The words in a file name: CinderBlockPath12 is Cinder, Block, Path, 12 */
   private static final Pattern NAME_WORD = Pattern.compile("[A-Z][a-z]*|[a-z]+|\\d+");
   private final Point3D tempIntersection = new Point3D();
   private final Point3D tempTranslation = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final ImBoolean inputsEnabled = new ImBoolean(false);
   private final ImString saveString = new ImString(256);
   private final ImString objectSearch = new ImString(64);
   private final ImString environmentSearch = new ImString(64);
   /** Rows the object list shows before it scrolls. */
   private static final int OBJECT_LIST_VISIBLE_ROWS = 8;

   private boolean loadedFilesOnce = false;
   private boolean isPlacing = false;
   private boolean confirmingDeleteAll = false;
   private String selectedEnvironmentFile = null;
   private String isaacSimExportStatus = null;
   private RDXEnvironmentObject selectedObject;
   private RDXEnvironmentObject intersectedObject;

   public RDXEnvironmentBuilder(RDX3DPanel panel3D)
   {
      super("Environment");
      this.panel3D = panel3D;
      setRenderMethod(this::renderImGuiWidgets);
      RDXBaseUI.getInstance().getKeyBindings().register("Delete selected object", "Delete");
      RDXBaseUI.getInstance().getKeyBindings().register("Deselect object", "Escape");
   }

   public void create()
   {
      pose3DGizmo.create(panel3D);
      panel3D.getScene().addRenderableProvider(this::getRenderables);
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (inputsEnabled.get())
      {
         if (selectedObject != null && !isPlacing)
         {
            pose3DGizmo.calculate3DViewPick(input);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput viewInput)
   {
      updateProceduralGround();

      if (selectedObject != null && ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
         resetSelection();

      if (inputsEnabled.get())
      {
         if (selectedObject != null)
         {
            if (isPlacing)
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
                  isPlacing = false;
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
            isPlacing = false;
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
   }

   private RDXEnvironmentObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      RDXEnvironmentObject closestObject = null;
      for (RDXEnvironmentObject object : allObjects)
      {
         if (!object.isSelectable())
            continue;
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

   public void resetSelection()
   {
      updateObjectSelected(selectedObject, null);
      intersectedObject = null;
   }

   public void update()
   {

   }

   public void renderImGuiWidgets()
   {
      ImGui.separator();

      if (ImGui.checkbox(labels.get("Enable Environment Building"), inputsEnabled) && !inputsEnabled.get())
         stopBuilding();

      ImGui.separator();
      ImGui.text("Selected Object: " + (selectedObject == null ? "" : (selectedObject.getTitleCasedName() + " " + selectedObject.getObjectIndex())));
      if (selectedObject != null)
         selectedObject.renderImGuiWidgets();
      ImGui.text("Highlighted Object: " + (intersectedObject == null ? "" : (intersectedObject.getTitleCasedName() + " " + intersectedObject.getObjectIndex())));

      if (ImGui.button("Delete selected object") && selectedObject != null
          || ImGui.isKeyReleased(ImGuiTools.getDeleteKey()) && !ImGui.getIO().getWantTextInput())
      {
         removeObject(selectedObject);
         resetSelection();
      }

      if (!confirmingDeleteAll)
      {
         if (ImGui.button(labels.get("Delete all objects")))
            confirmingDeleteAll = true;
      }
      else
      {
         ImGui.text("Delete all %d objects?".formatted(allObjects.size()));
         ImGui.sameLine();
         if (ImGui.button(labels.get("Confirm")))
         {
            removeAllObjects();
            // Unselect the file so "Save" can't overwrite it with an empty environment
            selectedEnvironmentFile = null;
            confirmingDeleteAll = false;
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Cancel")))
            confirmingDeleteAll = false;
      }

      ImGui.separator();

      if (inputsEnabled.get())
      {
         renderObjectList();
         ImGui.separator();
      }

      if (selectedObject instanceof RDXProceduralTerrainObject terrainObject)
      {
         terrainObject.renderImGuiWidgets();
         ImGui.separator();
      }

      ImGui.text("Environments:");
      if (!loadedFilesOnce && selectedEnvironmentFile != null)
      {
         loadEnvironmentInternal(selectedEnvironmentFile);
      }
      String fileNameToSave = null;
      boolean refreshClicked = renderEnvironmentList();
      if (!loadedFilesOnce || refreshClicked)
      {
         loadedFilesOnce = true;
         reindexScripts();
      }
      if (selectedEnvironmentFile != null)
      {
         if (ImGui.button(labels.get("Save " + selectedEnvironmentFile)))
            fileNameToSave = selectedEnvironmentFile;
      }

      ImGui.separator();
      ImGuiTools.inputText("###saveText", saveString);
      ImGui.sameLine();
      if (ImGui.button("Save as"))
      {
         fileNameToSave = saveString.get().trim();
         if (!fileNameToSave.endsWith(".json"))
            fileNameToSave += ".json";
      }
      if (fileNameToSave != null)
      {
         JSONFileTools.saveToClasspath("ihmc-open-robotics-software",
                                       "ihmc-high-level-behaviors/src/libgdx/resources",
                                       "environments/" + fileNameToSave,
         rootNode ->
         {
            rootNode.put("ambientLight", ambientLightAmount.get());
            ArrayNode objectsArrayNode = rootNode.putArray("objects");
            for (RDXEnvironmentObject object : allObjects)
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
               if (object instanceof RDXProceduralTerrainObject terrainObject)
                  terrainObject.saveParameters(objectNode);
            }
         });
         reindexScripts();
      }

      ImGui.separator();
      if (ImGui.button(labels.get("Export to Isaac Sim")))
         exportToIsaacSim();
      ImGuiTools.previousWidgetTooltip("Writes this environment as a USD stage Isaac Sim opens, under " + isaacSimExportRoot());
      if (isaacSimExportStatus != null)
         ImGui.textWrapped(isaacSimExportStatus);
   }

   /** {@code ~/.ihmc/isaacSimExports}, one folder per environment. */
   private static java.nio.file.Path isaacSimExportRoot()
   {
      return java.nio.file.Paths.get(System.getProperty("user.home"), ".ihmc", "isaacSimExports");
   }

   private void exportToIsaacSim()
   {
      String environmentName = selectedEnvironmentFile == null ? "Untitled" : selectedEnvironmentFile.replaceFirst("\\.json$", "");
      try
      {
         java.nio.file.Path stage = new RDXIsaacSimEnvironmentExporter().export(environmentName,
                                                                                allObjects,
                                                                                ambientLightAmount.get(),
                                                                                isaacSimExportRoot().resolve(environmentName));
         isaacSimExportStatus = "Exported " + stage;
      }
      catch (java.io.IOException | RuntimeException exception)
      {
         isaacSimExportStatus = "Isaac Sim export failed: " + exception.getMessage();
         LogTools.error(isaacSimExportStatus, exception);
      }
   }

   private static boolean isProceduralTerrain(RDXEnvironmentObjectFactory objectFactory)
   {
      return RDXProceduralTerrainObject.class.isAssignableFrom(objectFactory.getClazz());
   }

   /**
    * A search box over a short scrolling list of the placeable objects, sorted by name, with the procedural terrain
    * last. Clicking a row places that object.
    */
   private void renderObjectList()
   {
      ImGui.text("Place object:");
      ImGui.setNextItemWidth(-1.0f);
      ImGui.inputTextWithHint(labels.getHidden("Object search"), "Type to search", objectSearch);
      String search = objectSearch.get().trim().toLowerCase(Locale.ROOT);

      List<RDXEnvironmentObjectFactory> objects = new ArrayList<>();
      List<RDXEnvironmentObjectFactory> terrain = new ArrayList<>();
      for (RDXEnvironmentObjectFactory objectFactory : RDXEnvironmentObjectLibrary.getObjectFactories())
      {
         if (search.isEmpty() || objectFactory.getName().toLowerCase(Locale.ROOT).contains(search))
            (isProceduralTerrain(objectFactory) ? terrain : objects).add(objectFactory);
      }
      Comparator<RDXEnvironmentObjectFactory> byName = Comparator.comparing(RDXEnvironmentObjectFactory::getName, String.CASE_INSENSITIVE_ORDER);
      objects.sort(byName);
      terrain.sort(byName);

      int rows = objects.size() + terrain.size() + (terrain.isEmpty() ? 0 : 1);
      float rowHeight = ImGui.getTextLineHeightWithSpacing();
      float listHeight = rowHeight * Math.max(1, Math.min(rows, OBJECT_LIST_VISIBLE_ROWS)) + ImGui.getStyle().getWindowPaddingY() * 2.0f;
      if (ImGui.beginChild(labels.getHidden("Object list"), 0.0f, listHeight, true))
      {
         for (RDXEnvironmentObjectFactory objectFactory : objects)
            renderPlaceRow(objectFactory);
         if (!terrain.isEmpty())
         {
            ImGui.textDisabled("Procedural terrain");
            for (RDXEnvironmentObjectFactory objectFactory : terrain)
               renderPlaceRow(objectFactory);
         }
         if (rows == 0)
            ImGui.textDisabled("No objects match \"" + objectSearch.get().trim() + "\"");
      }
      ImGui.endChild();
   }

   /**
    * A search box with a refresh button over a short scrolling list of the environment files. Clicking a row loads
    * that environment.
    *
    * @return true when the refresh button was clicked
    */
   private boolean renderEnvironmentList()
   {
      float refreshButtonWidth = ImGui.getFrameHeight();
      ImGui.setNextItemWidth(-(refreshButtonWidth + ImGui.getStyle().getItemSpacingX()));
      ImGui.inputTextWithHint(labels.getHidden("Environment search"), "Type to search", environmentSearch);
      ImGui.sameLine();
      boolean refreshClicked = ImGuiTools.refreshButton(labels.getHidden("Refresh environments"));
      ImGuiTools.previousWidgetTooltip("Refresh the environment file list");
      String search = environmentSearch.get().trim().toLowerCase(Locale.ROOT);

      List<String> matches = new ArrayList<>();
      for (String environmentFileName : environmentFileNames)
      {
         if (search.isEmpty() || environmentFileName.toLowerCase(Locale.ROOT).contains(search))
            matches.add(environmentFileName);
      }

      // Rows in file order; a group takes the place of its first file
      Map<String, List<String>> groupMatches = new LinkedHashMap<>();
      List<String> rows = new ArrayList<>();
      for (String environmentFileName : matches)
      {
         String group = environmentGroups.get(environmentFileName);
         if (group == null)
         {
            rows.add(environmentFileName);
         }
         else
         {
            if (!groupMatches.containsKey(group))
               rows.add(group);
            groupMatches.computeIfAbsent(group, key -> new ArrayList<>()).add(environmentFileName);
         }
      }

      float rowHeight = ImGui.getTextLineHeightWithSpacing();
      float listHeight = rowHeight * Math.max(1, Math.min(matches.size(), OBJECT_LIST_VISIBLE_ROWS)) + ImGui.getStyle().getWindowPaddingY() * 2.0f;
      if (ImGui.beginChild(labels.getHidden("Environment list"), 0.0f, listHeight, true))
      {
         for (String row : rows)
         {
            List<String> groupFiles = groupMatches.get(row);
            if (groupFiles == null)
            {
               if (ImGui.selectable(labels.get(row, "Environment"), row.equals(selectedEnvironmentFile)))
                  loadEnvironmentInternal(row);
               continue;
            }

            boolean containsSelected = groupFiles.contains(selectedEnvironmentFile);
            if (!search.isEmpty())
               ImGui.setNextItemOpen(true, ImGuiCond.Always);
            else if (containsSelected)
               ImGui.setNextItemOpen(true, ImGuiCond.Once);
            int flags = ImGuiTreeNodeFlags.SpanAvailWidth | (containsSelected ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None);
            if (ImGui.treeNodeEx(labels.get(row, "EnvironmentGroup"), flags))
            {
               for (String environmentFileName : groupFiles)
               {
                  String variant = environmentVariantNames.get(environmentFileName);
                  if (ImGui.selectable(labels.get(variant, environmentFileName), environmentFileName.equals(selectedEnvironmentFile)))
                     loadEnvironmentInternal(environmentFileName);
                  if (ImGui.isItemHovered())
                     ImGui.setTooltip(environmentFileName);
               }
               ImGui.treePop();
            }
         }
         if (matches.isEmpty())
            ImGui.textDisabled(environmentFileNames.isEmpty() ? "No environment files" : "No environments match \"" + environmentSearch.get().trim() + "\"");
      }
      ImGui.endChild();
      return refreshClicked;
   }

   private void renderPlaceRow(RDXEnvironmentObjectFactory objectFactory)
   {
      if (ImGui.selectable(labels.get(objectFactory.getName(), "Place")))
         placeObject(objectFactory);
      if (ImGui.isItemHovered())
         ImGui.setTooltip("Place " + objectFactory.getName());
   }

   private void placeObject(RDXEnvironmentObjectFactory objectFactory)
   {
      if (isPlacing)
      {
         removeObject(selectedObject);
      }

      RDXEnvironmentObject objectToPlace = objectFactory.getSupplier().get();
      addObject(objectToPlace);
      updateObjectSelected(selectedObject, objectToPlace);

      if (objectToPlace instanceof RDXProceduralTerrainObject)
      {
         // Terrain is too large to drag with the mouse, so drop it on the ground under the camera focus and position it with the gizmo
         tempTranslation.set(panel3D.getCamera3D().getFocusPointPose().getPosition());
         tempTranslation.setZ(0.0);
         objectToPlace.setPositionInWorld(tempTranslation);
         pose3DGizmo.getTransformToParent().set(objectToPlace.getObjectTransform());
         isPlacing = false;
      }
      else
      {
         isPlacing = true;
      }
   }

   private void loadEnvironmentInternal(String environmentFileName)
   {
      loadedFilesOnce = true;
      selectedEnvironmentFile = environmentFileName;
      removeAllObjects();

      JSONFileTools.load(new WorkspaceResourceFile(environmentFilesDirectory, selectedEnvironmentFile),
      node ->
      {
         JsonNode ambientLightNode = node.get("ambientLight");
         if (ambientLightNode != null)
         {
            float ambientValue = (float) ambientLightNode.asDouble();
            ambientLightAmount.set(ambientValue);
            panel3D.getScene().setAmbientLightIntensity(ambientLightAmount.get());
         }
         JSONTools.forEachArrayElement(node, "objects", objectNode ->
         {
            String objectTypeName = objectNode.get("type").asText();
            RDXEnvironmentObject object = RDXEnvironmentObjectLibrary.loadBySimpleClassName(objectTypeName);

            if (object != null)
            {
               if (object instanceof RDXProceduralTerrainObject terrainObject)
                  terrainObject.loadParameters(objectNode);
               tempTranslation.setX(objectNode.get("x").asDouble());
               tempTranslation.setY(objectNode.get("y").asDouble());
               tempTranslation.setZ(objectNode.get("z").asDouble());
               tempOrientation.set(objectNode.get("qx").asDouble(),
                                   objectNode.get("qy").asDouble(),
                                   objectNode.get("qz").asDouble(),
                                   objectNode.get("qs").asDouble());
               tempTransform.set(tempOrientation, tempTranslation);
               object.setTransformToWorld(tempTransform);
               addObject(object);
            }
            else
            {
               LogTools.warn("Skipping loading object: {}", objectTypeName);
            }
         });
      });
      updateProceduralGround();
   }

   /** Cuts the footprints of the procedural terrain out of any procedural ground. Only rebuilds when they moved. */
   private void updateProceduralGround()
   {
      proceduralTerrainObjects.clear();
      for (RDXEnvironmentObject object : allObjects)
      {
         if (object instanceof RDXProceduralTerrainObject terrainObject && !(object instanceof RDXProceduralGroundObject))
            proceduralTerrainObjects.add(terrainObject);
      }
      for (RDXEnvironmentObject object : allObjects)
      {
         if (object instanceof RDXProceduralGroundObject groundObject)
            groundObject.updateHoles(proceduralTerrainObjects);
      }
   }

   public void removeAllObjects()
   {
      for (RDXEnvironmentObject object : allObjects.toArray(new RDXEnvironmentObject[0]))
      {
         removeObject(object);
      }
      isPlacing = false;
      resetSelection();
   }

   public void loadEnvironment(String environmentFileName)
   {
      LogTools.info("Loading environment: {}", environmentFileName);
      reindexScripts();
      Optional<String> match = environmentFileNames.stream().filter(fileName -> fileName.equals(environmentFileName)).findFirst();
      if (match.isPresent())
      {
         loadEnvironmentInternal(match.get());
      }
      else
      {
         LogTools.error("Could not find environment file: {}", environmentFileName);
      }
   }

   public void updateObjectSelected(RDXEnvironmentObject from, RDXEnvironmentObject to)
   {
      if (from != to)
      {
         if (from != null)
            from.setSelected(false);

         if (to != null)
            to.setSelected(true);

         selectedObject = to;
      }
   }

   /**
    * When enabled, left-click picks objects in the 3D view and the pose gizmo can move the selection.
    * Disabling it also clears the selection.
    */
   public void setInputsEnabled(boolean enabled)
   {
      inputsEnabled.set(enabled);
      if (!enabled)
         stopBuilding();
   }

   /**
    * Unselects everything, so the gizmo and the collision meshes go away. A selected object is also
    * one the simulation treats as being dragged, so leaving it selected would keep it pinned in place.
    */
   private void stopBuilding()
   {
      isPlacing = false;
      resetSelection();
   }

   public boolean getInputsEnabled()
   {
      return inputsEnabled.get();
   }

   public void selectObject(RDXEnvironmentObject object)
   {
      if (object != null && !object.isSelectable())
         return;
      updateObjectSelected(selectedObject, object);
      if (selectedObject != null)
         pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());
   }

   public void addObject(RDXEnvironmentObject environmentObject)
   {
      allObjects.add(environmentObject);

      if (environmentObject instanceof RDXPointLightObject pointLightObject)
      {
         panel3D.getScene().addPointLight(pointLightObject.getLight());
         lightObjects.add(pointLightObject);
      }
      else if (environmentObject instanceof RDXDirectionalLightObject directionalLightObject)
      {
         panel3D.getScene().addDirectionalLight(directionalLightObject.getLight());
         lightObjects.add(directionalLightObject);
      }
   }

   public void removeObject(RDXEnvironmentObject environmentObject)
   {
      if (environmentObject == null)
         return;
      allObjects.remove(environmentObject);
      environmentObject.removedFromEnvironment();

      if (environmentObject instanceof RDXPointLightObject lightObject)
      {
         panel3D.getScene().removePointLight(lightObject.getLight());
         lightObjects.remove(environmentObject);
      }
      else if (environmentObject instanceof RDXDirectionalLightObject lightObject)
      {
         panel3D.getScene().removeDirectionalLight(lightObject.getLight());
         lightObjects.remove(environmentObject);
      }
   }

   private void reindexScripts()
   {
      environmentFileNames.clear();
      environmentFilesDirectory.walkResourcesFlat((path, pathType) ->
      {
         if (pathType == BasicPathVisitor.PathType.FILE)
         {
            environmentFileNames.add(path);
         }
      });
      groupEnvironmentFiles();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.GROUND_TRUTH))
      {
         for (RDXEnvironmentObject object : allObjects)
         {
            if (!(object instanceof RDXPointLightObject) && !(object instanceof RDXDirectionalLightObject))
               object.getRealRenderables(renderables, pool);
         }
      }
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         for (RDXEnvironmentObject object : lightObjects)
         {
            object.getRealRenderables(renderables, pool);
         }
         // Selection graphics are building tools, so they only show while building is enabled
         if (inputsEnabled.get() && selectedObject != null)
         {
            selectedObject.getCollisionMeshRenderables(renderables, pool);
            pose3DGizmo.getRenderables(renderables, pool);
         }
         if (inputsEnabled.get() && intersectedObject != null && intersectedObject != selectedObject)
         {
            intersectedObject.getCollisionMeshRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {

   }

   public ArrayList<RDXEnvironmentObject> getAllObjects()
   {
      return allObjects;
   }

   /** Alphabetical order, with digit runs compared as numbers so 2 comes before 10. */
   /**
    * Files that share their leading words with other files form one expandable row: CinderBlockPath1.json to
    * CinderBlockPath13.json become CinderBlockPath with rows 1 to 13, and EmptyRoomPullDoor.json and
    * EmptyRoomPushDoor.json become EmptyRoom with rows Pull Door and Push Door. A file named just the shared words,
    * like Stairs.json next to StairsAndCinderBlockPath.json, keeps them apart.
    */
   private void groupEnvironmentFiles()
   {
      environmentGroups.clear();
      environmentVariantNames.clear();

      Map<String, List<String>> words = new HashMap<>();
      Set<String> baseNames = new HashSet<>();
      for (String environmentFileName : environmentFileNames)
      {
         String baseName = environmentFileName.endsWith(".json") ? environmentFileName.substring(0, environmentFileName.length() - 5) : environmentFileName;
         baseNames.add(baseName);
         List<String> fileWords = new ArrayList<>();
         Matcher matcher = NAME_WORD.matcher(baseName);
         while (matcher.find())
            fileWords.add(matcher.group());
         words.put(environmentFileName, fileWords);
      }

      for (String environmentFileName : environmentFileNames)
      {
         List<String> fileWords = words.get(environmentFileName);
         for (int length = fileWords.size() - 1; length >= 1; length--)
         {
            List<String> prefix = fileWords.subList(0, length);
            String group = String.join("", prefix);
            if (baseNames.contains(group))
               continue;
            boolean shared = false;
            for (Map.Entry<String, List<String>> other : words.entrySet())
            {
               if (!other.getKey().equals(environmentFileName) && other.getValue().size() > length && other.getValue().subList(0, length).equals(prefix))
               {
                  shared = true;
                  break;
               }
            }
            if (shared)
            {
               environmentGroups.put(environmentFileName, group);
               environmentVariantNames.put(environmentFileName, String.join(" ", fileWords.subList(length, fileWords.size())));
               break;
            }
         }
      }
   }

   private static int compareFileNames(String left, String right)
   {
      int i = 0;
      int j = 0;
      while (i < left.length() && j < right.length())
      {
         if (Character.isDigit(left.charAt(i)) && Character.isDigit(right.charAt(j)))
         {
            int leftStart = i;
            int rightStart = j;
            while (i < left.length() && Character.isDigit(left.charAt(i)))
               i++;
            while (j < right.length() && Character.isDigit(right.charAt(j)))
               j++;
            int numberComparison = compareNumericText(left.substring(leftStart, i), right.substring(rightStart, j));
            if (numberComparison != 0)
               return numberComparison;
         }
         else
         {
            int characterComparison = Character.compare(Character.toLowerCase(left.charAt(i)), Character.toLowerCase(right.charAt(j)));
            if (characterComparison != 0)
               return characterComparison;
            i++;
            j++;
         }
      }
      int lengthComparison = Integer.compare(left.length() - i, right.length() - j);
      return lengthComparison != 0 ? lengthComparison : left.compareTo(right);
   }

   private static int compareNumericText(String left, String right)
   {
      int leftIndex = 0;
      int rightIndex = 0;
      while (leftIndex < left.length() - 1 && left.charAt(leftIndex) == '0')
         leftIndex++;
      while (rightIndex < right.length() - 1 && right.charAt(rightIndex) == '0')
         rightIndex++;
      int lengthComparison = Integer.compare(left.length() - leftIndex, right.length() - rightIndex);
      if (lengthComparison != 0)
         return lengthComparison;
      int digitComparison = left.substring(leftIndex).compareTo(right.substring(rightIndex));
      return digitComparison != 0 ? digitComparison : Integer.compare(left.length(), right.length());
   }
}
