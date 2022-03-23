package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
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
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.bullet.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectLibrary;
import us.ihmc.gdx.simulation.environment.object.objects.*;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.*;

public class GDXEnvironmentBuilder extends ImGuiPanel
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironmentBuilder.class, "Environment");
   private final ArrayList<GDXEnvironmentObject> allObjects = new ArrayList<>();
   private final ArrayList<GDXEnvironmentObject> lightObjects = new ArrayList<>();
   private boolean loadedFilesOnce = false;
   private Path selectedEnvironmentFile = null;
   private final TreeSet<Path> environmentFiles = new TreeSet<>(Comparator.comparing(path -> path.getFileName().toString()));
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString saveString = new ImString("", 100);
   private final Point3D tempTranslation = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final GDX3DSceneManager sceneManager;
   private boolean isPlacing = false;
   private GDXEnvironmentObject selectedObject;
   private GDXEnvironmentObject intersectedObject;
   private final GDXPose3DGizmo pose3DGizmo = new GDXPose3DGizmo();
   private final ImGuiPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());
   private final Point3D tempIntersection = new Point3D();
   private final GDXBulletPhysicsManager bulletPhysicsManager = new GDXBulletPhysicsManager();

   public GDXEnvironmentBuilder(GDX3DSceneManager sceneManager)
   {
      super(WINDOW_NAME);
      this.sceneManager = sceneManager;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      bulletPhysicsManager.create();

      // TODO: Implement hiding the real environment to emulate real world operation
      sceneManager.addRenderableProvider(this::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      sceneManager.addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      pose3DGizmo.create(sceneManager.getCamera3D());
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void process3DViewInput(ImGui3DViewInput viewInput)
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

            selectedObject.copyThisTransformToBulletMultiBody();

            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               isPlacing = false;
            }
         }
         else
         {
            pose3DGizmo.process3DViewInput(viewInput);
            selectedObject.setTransformToWorld(pose3DGizmo.getTransformToParent());

            selectedObject.copyThisTransformToBulletMultiBodyParentOnly();

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

   public void resetSelection()
   {
      updateObjectSelected(selectedObject, null);
      intersectedObject = null;
   }

   public void update()
   {
      bulletPhysicsManager.simulate(Gdx.graphics.getDeltaTime());
      for (GDXEnvironmentObject allObject : allObjects)
      {
         if (bulletPhysicsManager.getSimulate().get())
         {
            allObject.copyBulletTransformToThisMultiBody();
            allObject.afterSimulate(bulletPhysicsManager);
         }
         allObject.update(bulletPhysicsManager);
      }
   }

   public void renderImGuiWidgets()
   {
      bulletPhysicsManager.renderImGuiWidgets();

      ImGui.separator();
      if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
      {
         sceneManager.getSceneBasics().setAmbientLight(ambientLightAmount.get());
      }

      ImGui.separator();
      ImGui.text("Selected: " + (selectedObject == null ? "" : (selectedObject.getTitleCasedName() + " " + selectedObject.getObjectIndex())));
      ImGui.text("Intersected: " + (intersectedObject == null ? "" : (intersectedObject.getTitleCasedName() + " " + intersectedObject.getObjectIndex())));

      // TODO: Place robots
      if (!isPlacing)
      {
         for (GDXEnvironmentObjectFactory objectFactory : GDXEnvironmentObjectLibrary.getObjectFactories())
         {
            if (ImGui.button(labels.get("Place " + objectFactory.getName())))
            {
               GDXEnvironmentObject objectToPlace = objectFactory.getSupplier().get();
               addObject(objectToPlace);
               updateObjectSelected(selectedObject, objectToPlace);
               isPlacing = true;
            }
         }

         ImGui.separator();
      }
      if (selectedObject != null && (ImGui.button("Delete selected") || ImGui.isKeyReleased(ImGuiTools.getDeleteKey())))
      {
         removeObject(selectedObject);
         resetSelection();
      }

      ImGui.text("Environments:");
      if (!loadedFilesOnce && selectedEnvironmentFile != null)
      {
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
            rootNode.put("ambientLight", ambientLightAmount.get());
            ArrayNode objectsArrayNode = rootNode.putArray("objects");
            for (GDXEnvironmentObject object : allObjects)
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

      ImGui.checkbox("Show 3D Widget Tuner", poseGizmoTunerPanel.getIsShowing());
   }

   private void loadEnvironment(Path environmentFile)
   {
      loadedFilesOnce = true;
      selectedEnvironmentFile = environmentFile;
      bulletPhysicsManager.getSimulate().set(false);
      for (GDXEnvironmentObject object : allObjects.toArray(new GDXEnvironmentObject[0]))
      {
         removeObject(object);
      }

      resetSelection();

      JSONFileTools.loadFromWorkspace("ihmc-open-robotics-software",
                                      "ihmc-high-level-behaviors/src/libgdx/resources",
                                      "environments/" + environmentFile.getFileName().toString(),
      node ->
      {
         JsonNode ambientLightNode = node.get("ambientLight");
         if (ambientLightNode != null)
         {
            float ambientValue = (float) ambientLightNode.asDouble();
            ambientLightAmount.set(ambientValue);
            sceneManager.getSceneBasics().setAmbientLight(ambientLightAmount.get());
         }
         for (Iterator<JsonNode> it = node.withArray("objects").elements(); it.hasNext(); )
         {
            JsonNode objectNode = it.next();
            String objectTypeName = objectNode.get("type").asText();
            GDXEnvironmentObject object = GDXEnvironmentObjectLibrary.loadBySimpleClassName(objectTypeName);

            if (object != null)
            {
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
               object.copyThisTransformToBulletMultiBody();
            }
            else
            {
               LogTools.warn("Skipping loading object: {}", objectTypeName);
            }
         }
      });
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

   public void updateObjectSelected(GDXEnvironmentObject from, GDXEnvironmentObject to)
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

   public void addObject(GDXEnvironmentObject environmentObject)
   {
      allObjects.add(environmentObject);
      environmentObject.addToBullet(bulletPhysicsManager);

      if (environmentObject instanceof GDXPointLightObject)
      {
         GDXPointLightObject pointLightObject = (GDXPointLightObject) environmentObject;
         sceneManager.getSceneBasics().addPointLight(pointLightObject.getLight());
         lightObjects.add(pointLightObject);
      }
      else if (environmentObject instanceof GDXDirectionalLightObject)
      {
         GDXDirectionalLightObject directionalLightObject = (GDXDirectionalLightObject) environmentObject;
         sceneManager.getSceneBasics().addDirectionalLight(directionalLightObject.getLight());
         lightObjects.add(directionalLightObject);
      }
   }

   public void removeObject(GDXEnvironmentObject environmentObject)
   {
      allObjects.remove(environmentObject);
      environmentObject.removeFromBullet();

      if (environmentObject instanceof GDXPointLightObject)
      {
         GDXPointLightObject lightObject = (GDXPointLightObject) environmentObject;
         sceneManager.getSceneBasics().removePointLight(lightObject.getLight());
         lightObjects.remove(environmentObject);
      }
      else if (environmentObject instanceof GDXDirectionalLightObject)
      {
         GDXDirectionalLightObject lightObject = (GDXDirectionalLightObject) environmentObject;
         sceneManager.getSceneBasics().removeDirectionalLight(lightObject.getLight());
         lightObjects.remove(environmentObject);
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

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXEnvironmentObject object : allObjects)
      {
         if (!(object instanceof GDXPointLightObject) && !(object instanceof GDXDirectionalLightObject))
            object.getRealRenderables(renderables, pool);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXEnvironmentObject object : lightObjects)
      {
         object.getRealRenderables(renderables, pool);
      }
      if (selectedObject != null)
      {
         selectedObject.getCollisionMeshRenderables(renderables, pool);
         pose3DGizmo.getRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionMeshRenderables(renderables, pool);
      }
      bulletPhysicsManager.getVirtualRenderables(renderables, pool);
   }

   public void destroy()
   {
      bulletPhysicsManager.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public GDXBulletPhysicsManager getBulletPhysicsManager()
   {
      return bulletPhysicsManager;
   }

   public ArrayList<GDXEnvironmentObject> getAllObjects()
   {
      return allObjects;
   }
}
