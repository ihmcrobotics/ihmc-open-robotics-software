package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectLibrary;
import us.ihmc.gdx.simulation.environment.object.objects.*;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.*;
import java.util.function.Supplier;

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
   private final GDXEnvironmentObjectInteraction objectInteraction = new GDXEnvironmentObjectInteraction();

   public GDXEnvironmentBuilder(GDX3DSceneManager sceneManager)
   {
      super(WINDOW_NAME);
      this.sceneManager = sceneManager;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(objectInteraction.getPoseGizmoTunerPanel());
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      // TODO: Implement hiding the real environment to emulate real world operation
      sceneManager.addRenderableProvider(this::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
      sceneManager.addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      objectInteraction.create(baseUI.get3DSceneManager(), allObjects, lightObjects);
      baseUI.addImGui3DViewInputProcessor(objectInteraction::process3DViewInput);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Selected: " + objectInteraction.getSelectedObject());
      ImGui.text("Intersected: " + objectInteraction.getIntersectedObject());

      GDXEnvironmentObject objectToPlace = null;
      if (!objectInteraction.isPlacing())
      {
         for (GDXEnvironmentObjectFactory objectFactory : GDXEnvironmentObjectLibrary.getObjectFactories())
         {
            if (ImGui.button(labels.get("Place " + objectFactory.getName())))
            {
               objectToPlace = objectFactory.getSupplier().get();

               if (objectFactory.getClazz().equals(GDXPointLightObject.class))
               {
                  GDXPointLightObject pointLight = (GDXPointLightObject) objectToPlace;
                  sceneManager.getSceneBasics().addPointLight(pointLight.getLight());
               }
               else if (objectFactory.getClazz().equals(GDXDirectionalLightObject.class))
               {
                  GDXDirectionalLightObject directionalLight = (GDXDirectionalLightObject) objectToPlace;
                  sceneManager.getSceneBasics().addDirectionalLight(directionalLight.getLight());
               }
            }
         }

         ImGui.separator();

         if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
         {
            sceneManager.getSceneBasics().setAmbientLight(ambientLightAmount.get());
         }

         ImGui.separator();
      }
      objectInteraction.handleObjectPlacementAndRemoval(objectToPlace);

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
            for (GDXEnvironmentObject object : this.allObjects)
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

      ImGui.checkbox("Show 3D Widget Tuner", objectInteraction.getPoseGizmoTunerPanel().getIsShowing());
   }

   private void loadEnvironment(Path environmentFile)
   {
      loadedFilesOnce = true;
      selectedEnvironmentFile = environmentFile;
      allObjects.clear();

      sceneManager.getSceneBasics().clearLights();
      lightObjects.clear();

      objectInteraction.resetSelection();

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
            GDXEnvironmentObject object = GDXEnvironmentObjectLibrary.loadBySimpleClassName(objectNode.get("type").asText());

            tempTranslation.setX(objectNode.get("x").asDouble());
            tempTranslation.setY(objectNode.get("y").asDouble());
            tempTranslation.setZ(objectNode.get("z").asDouble());
            tempOrientation.set(objectNode.get("qx").asDouble(),
                                objectNode.get("qy").asDouble(),
                                objectNode.get("qz").asDouble(),
                                objectNode.get("qs").asDouble());
            tempTransform.set(tempOrientation, tempTranslation);
            object.set(tempTransform);
            allObjects.add(object);

            if (object instanceof GDXPointLightObject)
            {
               GDXPointLightObject pointLightObject = (GDXPointLightObject) object;
               sceneManager.getSceneBasics().addPointLight(pointLightObject.getLight());
               lightObjects.add(pointLightObject);
            }
            else if (object instanceof GDXDirectionalLightObject)
            {
               GDXDirectionalLightObject directionalLightObject = (GDXDirectionalLightObject) object;
               sceneManager.getSceneBasics().addDirectionalLight(directionalLightObject.getLight());
               lightObjects.add(directionalLightObject);
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
            object.getRealisticModelInstance().getRenderables(renderables, pool);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXEnvironmentObject object : lightObjects)
      {
         object.getRealisticModelInstance().getRenderables(renderables, pool);
      }

      objectInteraction.getVirtualRenderables(renderables, pool);
   }

   public void destroy()
   {

   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
