package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.object.RDXSimpleObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXBuildingObject;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.List;

public class RDXBuildingConstructor extends RDXPanel
{
   private enum Mode
   {
      NONE, CONSTRUCTING, PLACING, DONE
   }

   private String selectedFileName = "";
   private final WorkspaceResourceDirectory environmentFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/buildings");
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(RDXEnvironmentBuilder.class, "Constructor");
   private final ArrayList<RDXSimpleObject> virtualObjects = new ArrayList<>();
   private RDXSimpleObject selectedObject;
   private RDXSimpleObject intersectedObject;
   private final ImFloat ambientLightAmount = new ImFloat(0.4f);
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();
   private final RDXPanel poseGizmoTunerPanel = pose3DGizmo.createTunerPanel(getClass().getSimpleName());

   private final RDX3DPanel panel3D;
   private final Point3D tempIntersection = new Point3D();

   private Point3D lastPickPoint = new Point3D();

   private Point3D cornerPoint;

   private final ColorAttribute highlightColor = ColorAttribute.createDiffuse(0.8f, 0.6f, 0.2f, 1.0f);
   List<WorkspaceResourceFile> buildingFilesInDirectory = new ArrayList<WorkspaceResourceFile>();

   private RDXBuildingObject building;
   private RDXSimpleObject lastWallBase;
   private Mode mode = Mode.NONE;

   public RDXBuildingConstructor(RDX3DPanel panel3D)
   {
      super(WINDOW_NAME);
      this.panel3D = panel3D;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);

      buildingFilesInDirectory = environmentFilesDirectory.queryContainedFiles();
   }

   public void create()
   {
      panel3D.getScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
      pose3DGizmo.create(panel3D);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXSimpleObject model : virtualObjects)
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

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (building != null)
      {
         for (RDXSimpleObject model : building.getAllObjects())
         {
            model.getRealRenderables(renderables, pool);
         }
      }
      if (selectedObject != null)
      {
         pose3DGizmo.getRenderables(renderables, pool);
         selectedObject.getCollisionMeshRenderables(renderables, pool);
      }
      if (intersectedObject != null && intersectedObject != selectedObject)
      {
         intersectedObject.getCollisionMeshRenderables(renderables, pool);
      }
   }

   public void process3DViewInput(ImGui3DViewInput viewInput)
   {
      constructionUpdate(viewInput);
      if (selectedObject != null)
      {
         if (mode == Mode.PLACING)
         {
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
            {
               mode = Mode.DONE;
            }
            if (viewInput.isWindowHovered() && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               building.addCorner(cornerPoint);
               mode = Mode.CONSTRUCTING;
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
         mode = Mode.NONE;
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

   private RDXSimpleObject calculatePickedObject(Line3DReadOnly pickRay)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      RDXSimpleObject closestObject = null;

      if (building != null)
      {
         for (RDXSimpleObject object : building.getAllObjects())
         {
            boolean intersects = object.intersect(pickRay, tempIntersection);
            double distance = tempIntersection.distance(pickRay.getPoint());
            if (intersects && (closestObject == null || distance < closestDistance))
            {
               closestObject = object;
               closestDistance = distance;
            }
         }
      }
      return closestObject;
   }

   private void constructionUpdate(ImGui3DViewInput viewInput)
   {

      if (mode != Mode.NONE)
      {
         Line3DReadOnly pickRay = viewInput.getPickRayInWorld();
         lastPickPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(EuclidCoreTools.origin3D,
                                                                                 Axis3D.Z,
                                                                                 pickRay.getPoint(),
                                                                                 pickRay.getDirection());
         switch (mode)
         {
            case CONSTRUCTING:
            {
               RDXSimpleObject objectToPlace = new RDXSimpleObject("Corner");
               Model objectModel = RDXModelBuilder.createCylinder(0.15f, 0.25f, new Point3D(), Color.BROWN).model;
               Box3D collisionBox = new Box3D(0.25f, 0.25f, 0.15f);
               objectToPlace.setRealisticModel(objectModel);
               //               objectToPlace.setCollisionModel(objectModel);
               objectToPlace.setCollisionGeometryObject(collisionBox);
               objectToPlace.setCollisionModelColor(highlightColor, 0.2f);
               virtualObjects.add(objectToPlace);
               updateObjectSelected(selectedObject, objectToPlace);

               mode = Mode.PLACING;
               break;
            }
            case PLACING:
            {
               cornerPoint = building.getClosestRectangularCorner(lastPickPoint);

               selectedObject.setPositionInWorld(cornerPoint);
               pose3DGizmo.getTransformToParent().set(selectedObject.getObjectTransform());

               break;
            }
            case DONE:
            {
               building.construct();

               selectedObject = null;
               virtualObjects.clear();
               mode = Mode.NONE;
               break;
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.separator();
      if (ImGui.sliderFloat("Ambient light", ambientLightAmount.getData(), 0.0f, 1.0f))
      {
         panel3D.getScene().setAmbientLightIntensity(ambientLightAmount.get());
      }
      ImGui.separator();
      if (mode == Mode.NONE)
      {
         if (ImGui.button("Create Building"))
         {
            building = new RDXBuildingObject();
            mode = Mode.CONSTRUCTING;
         }

         if (ImGui.button("Create Stairs"))
         {
            constructStairwell(3, 4, 10, 3.0f, 3.0f, 12.0f);
         }

         if (ImGui.button("Save Building"))
         {
            LogTools.info("Saving building to JSON: {}", "building.json");
            saveAsJSON("building.json");
         }

         if (ImGui.button("Load Building"))
         {
            LogTools.info("Loading building from JSON: {}", "building.json");
            loadFromJSON("building.json");
            LogTools.info("Building Found with {} Corners", building.getCorners().size());
         }

         if (ImGui.button("Reset Building"))
         {
            building = null;
         }

         for (WorkspaceResourceFile buildingFileName : buildingFilesInDirectory)
         {
            if (ImGui.radioButton(buildingFileName.getFileName(), selectedFileName.equals(buildingFileName.getFileName())))
            {
               loadFromJSON(buildingFileName.getFileName());
            }
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

   public void updateObjectSelected(RDXSimpleObject from, RDXSimpleObject to)
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

   public void constructStairwell(int floors, int sides, int numberOfStepsPerSide, float length, float width, float height)
   {
      if (building == null)
         return;

      float[] xOffsets = {-width / 2.0f, length / 1.4f, width / 2.0f, -length / 1.4f};
      float[] yOffsets = {width / 1.4f, length / 2.0f, -width / 1.4f, -length / 2.0f};

      float[] xPlatformOffsets = {-width / 2.0f, length / 2.0f, width / 2.0f, -length / 2.0f};
      float[] yPlatformOffsets = {width / 2.0f, length / 2.0f, -width / 2.0f, -length / 2.0f};

      for (int i = 0; i < floors; i++)
      {
         for (int j = 0; j < sides; j++)
         {
            RDXSimpleObject stairsObject = new RDXSimpleObject("Stairs_" + ((sides * i) + j));
            Model objectModel = RDXModelBuilder.createStairs(1.5f, 0.3f, 0.3f, 10, Color.GRAY).model;
            stairsObject.setRealisticModel(objectModel);
            stairsObject.setCollisionModel(objectModel);

            Box3D collisionBox = new Box3D(1.0f, 0.1f, building.getHeight());
            stairsObject.setCollisionGeometryObject(collisionBox);
            stairsObject.getRealisticModelOffset().getRotation().appendYawRotation(-2.0f * Math.PI / (float) (sides) * j);

            stairsObject.getRealisticModelOffset().getTranslation().add(xOffsets[j], yOffsets[j], 3.0f * (float) (i * sides + j) - 0.3f);
            stairsObject.setPositionInWorld(new Point3D(0.0f, 0.0f, 0.0f));

            building.insertComponent(RDXBuildingObject.ComponentType.STAIRS, stairsObject);

            RDXSimpleObject platformObject = new RDXSimpleObject("Platform_" + ((sides * i) + j));
            Model platformModel = RDXModelBuilder.createBox(3.0f, 1.5f, 0.3f, Color.GRAY).model;
            platformObject.setRealisticModel(platformModel);
            platformObject.setCollisionModel(platformModel);

            Box3D platformCollisionBox = new Box3D(1.0f, 0.1f, building.getHeight());
            platformObject.setCollisionGeometryObject(platformCollisionBox);
            platformObject.getRealisticModelOffset().getRotation().appendYawRotation(-2.0f * Math.PI / (float) (sides) * j);

            platformObject.getRealisticModelOffset().getTranslation().add(xOffsets[j], yOffsets[j], 3.0f * (float) (i * sides + j) - 0.45f);
            platformObject.setPositionInWorld(new Point3D(0.0f, 0.0f, 0.0f));

            building.insertComponent(RDXBuildingObject.ComponentType.PLATFORMS, platformObject);
         }
      }
   }

   public void saveAsJSON(String fileNameToSave)
   {
      JSONFileTools.saveToClasspath("ihmc-open-robotics-software",
                                    "ihmc-high-level-behaviors/src/libgdx/resources",
                                    "environments/" + fileNameToSave,
                                    rootNode ->
                                    {
                                       ArrayNode objectsArrayNode = rootNode.putArray("objects");
                                       for (Point3D corner : building.getCorners())
                                       {
                                          ObjectNode objectNode = objectsArrayNode.addObject();
                                          objectNode.put("type", "Corner");
                                          objectNode.put("x", corner.getX());
                                          objectNode.put("y", corner.getY());
                                          objectNode.put("z", corner.getZ());
                                       }
                                    });
   }

   public void loadFromJSON(String fileNameToLoad)
   {
      selectedFileName = fileNameToLoad;
      building = new RDXBuildingObject();
      JSONFileTools.load(new WorkspaceResourceFile(environmentFilesDirectory, fileNameToLoad), rootNode ->
      {
         JSONTools.forEachArrayElement(rootNode, "objects", objectNode ->
         {
            float x = objectNode.get("x").floatValue();
            float y = objectNode.get("y").floatValue();
            float z = objectNode.get("z").floatValue();

            building.addCorner(new Point3D(x, y, z));
         });
      });

      building.construct();
   }
}
