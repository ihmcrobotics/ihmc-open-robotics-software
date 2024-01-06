package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiMouseButton;
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
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectLibrary;
import us.ihmc.rdx.simulation.environment.object.objects.RDXDirectionalLightObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXPointLightObject;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;

public class RDXEnvironmentBuilder extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final WorkspaceResourceDirectory environmentFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/environments");
   private final ArrayList<RDXEnvironmentObject> allObjects = new ArrayList<>();
   private final ArrayList<RDXEnvironmentObject> lightObjects = new ArrayList<>();
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();
   private final RDX3DPanel panel3D;
   private final RDXPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());
   private final RDXBulletPhysicsManager bulletPhysicsManager = new RDXBulletPhysicsManager();

   private final TreeSet<String> environmentFileNames = new TreeSet<>();
   private final Point3D tempIntersection = new Point3D();
   private final Point3D tempTranslation = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final ImBoolean inputsEnabled = new ImBoolean(true);
   private final ImString saveString = new ImString(256);

   private boolean loadedFilesOnce = false;
   private boolean isPlacing = false;
   private String selectedEnvironmentFile = null;
   private RDXEnvironmentObject selectedObject;
   private RDXEnvironmentObject intersectedObject;

   public RDXEnvironmentBuilder(RDX3DPanel panel3D)
   {
      super("Environment");
      this.panel3D = panel3D;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);
      RDXBaseUI.getInstance().getKeyBindings().register("Delete selected object", "Delete");
   }

   public void create()
   {
      bulletPhysicsManager.create();

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
   }

   private RDXEnvironmentObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      RDXEnvironmentObject closestObject = null;
      for (RDXEnvironmentObject object : allObjects)
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
      for (RDXEnvironmentObject allObject : allObjects)
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

      ImGui.checkbox(labels.get("Enable Environment Building"), inputsEnabled);

      if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
      {
         panel3D.getScene().setAmbientLight(ambientLightAmount.get());
      }

      ImGui.separator();
      ImGui.text("Selected Object: " + (selectedObject == null ? "" : (selectedObject.getTitleCasedName() + " " + selectedObject.getObjectIndex())));
      ImGui.text("Highlighted Object: " + (intersectedObject == null ? "" : (intersectedObject.getTitleCasedName() + " " + intersectedObject.getObjectIndex())));

      if (ImGui.button("Delete selected object") && selectedObject != null || ImGui.isKeyReleased(ImGuiTools.getDeleteKey()))
      {
         removeObject(selectedObject);
         resetSelection();
      }

      ImGui.separator();

      if (inputsEnabled.get())
      {
         for (RDXEnvironmentObjectFactory objectFactory : RDXEnvironmentObjectLibrary.getObjectFactories())
         {
            if (ImGui.button(labels.get("Place " + objectFactory.getName())))
            {
               if (isPlacing)
               {
                  removeObject(selectedObject);
               }

               RDXEnvironmentObject objectToPlace = objectFactory.getSupplier().get();
               addObject(objectToPlace);
               updateObjectSelected(selectedObject, objectToPlace);
               isPlacing = true;
            }
         }

         ImGui.separator();
      }

      ImGui.text("Environments:");
      if (!loadedFilesOnce && selectedEnvironmentFile != null)
      {
         loadEnvironmentInternal(selectedEnvironmentFile);
      }
      boolean reindexClicked = ImGui.button(labels.get("Reindex scripts"));
      if (!loadedFilesOnce || reindexClicked)
      {
         loadedFilesOnce = true;
         reindexScripts();
      }
      String fileNameToSave = null;
      for (String environmentFileName : environmentFileNames)
      {
         if (ImGui.radioButton(environmentFileName, selectedEnvironmentFile != null && selectedEnvironmentFile.equals(environmentFileName)))
         {
            loadEnvironmentInternal(environmentFileName);
         }
         if (selectedEnvironmentFile != null && selectedEnvironmentFile.equals(environmentFileName))
         {
            ImGui.sameLine();
            if (ImGui.button("Save"))
            {
               fileNameToSave = environmentFileName;
            }
         }
      }

      ImGui.separator();
      ImGuiTools.inputText("###saveText", saveString);
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
            }
         });
         reindexScripts();
      }

      ImGui.checkbox("Show 3D Widget Tuner", poseGizmoTunerPanel.getIsShowing());
   }

   private void loadEnvironmentInternal(String environmentFileName)
   {
      loadedFilesOnce = true;
      selectedEnvironmentFile = environmentFileName;
      bulletPhysicsManager.getSimulate().set(false);
      for (RDXEnvironmentObject object : allObjects.toArray(new RDXEnvironmentObject[0]))
      {
         removeObject(object);
      }

      resetSelection();

      JSONFileTools.load(new WorkspaceResourceFile(environmentFilesDirectory, selectedEnvironmentFile),
      node ->
      {
         JsonNode ambientLightNode = node.get("ambientLight");
         if (ambientLightNode != null)
         {
            float ambientValue = (float) ambientLightNode.asDouble();
            ambientLightAmount.set(ambientValue);
            panel3D.getScene().setAmbientLight(ambientLightAmount.get());
         }
         JSONTools.forEachArrayElement(node, "objects", objectNode ->
         {
            String objectTypeName = objectNode.get("type").asText();
            RDXEnvironmentObject object = RDXEnvironmentObjectLibrary.loadBySimpleClassName(objectTypeName);

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
         });
      });
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

   public void addObject(RDXEnvironmentObject environmentObject)
   {
      allObjects.add(environmentObject);
      environmentObject.addToBullet(bulletPhysicsManager);

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
      allObjects.remove(environmentObject);
      environmentObject.removeFromBullet();

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
   }

   public void destroy()
   {
      bulletPhysicsManager.destroy();
   }

   public RDXBulletPhysicsManager getBulletPhysicsManager()
   {
      return bulletPhysicsManager;
   }

   public ArrayList<RDXEnvironmentObject> getAllObjects()
   {
      return allObjects;
   }

   public ImBoolean getInputsEnabled()
   {
      return inputsEnabled;
   }
}
