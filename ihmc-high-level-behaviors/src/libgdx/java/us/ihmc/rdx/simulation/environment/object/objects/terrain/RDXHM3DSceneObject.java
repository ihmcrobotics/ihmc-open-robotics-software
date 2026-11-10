package us.ihmc.rdx.simulation.environment.object.objects.terrain;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Color;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

/**
 * A scanned Habitat-Matterport 3D (HM3D) building: the scan as the visual, and boxes from
 * {@code scripts/hm3d_to_rdx.py} as the MuJoCo collision, so the robot can walk its floors and stairs.
 * <p>
 * Scenes live in {@code environmentObjects/hm3d/<scene>/}, which is git-ignored because HM3D is
 * licensed for academic, non-commercial use only. Run the script to add one. The object frame is
 * centered on the building footprint with z = 0 at its lowest floor.
 * </p>
 */
public class RDXHM3DSceneObject extends RDXProceduralTerrainObject
{
   public static final String NAME = "HM3D Scene";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXHM3DSceneObject.class);
   private static final String SCENES_DIRECTORY = "environmentObjects/hm3d/";

   private final String[] availableScenes = findScenes();
   private final ImInt selectedScene = new ImInt(0);
   private String loadedScene;
   private Box3D boundingBox;
   private final double[] bounds = {1.0, 1.0, 0.0, 1.0};
   private final Point3D firstIntersection = new Point3D();
   private final Point3D secondIntersection = new Point3D();
   /** Ground floor openings to the outside from the converter: x, y, outward yaw, width, in the object frame. */
   private final ArrayList<double[]> entrances = new ArrayList<>();
   private String[] entranceLabels = new String[0];
   private final ImInt selectedEntrance = new ImInt(0);
   private final ImDouble entranceDistance = new ImDouble(1.5);

   public RDXHM3DSceneObject()
   {
      super(NAME, FACTORY);
      rebuild();
   }

   private static String[] findScenes()
   {
      FileHandle index = Gdx.files.internal(SCENES_DIRECTORY + "scenes.txt");
      if (!index.exists())
         return new String[0];
      return Arrays.stream(index.readString().split("\\s+")).filter(scene -> !scene.isBlank()).toArray(String[]::new);
   }

   private String sceneId()
   {
      return availableScenes.length == 0 ? null : availableScenes[Math.min(selectedScene.get(), availableScenes.length - 1)];
   }

   @Override
   public void rebuild()
   {
      String scene = sceneId();
      if (scene == null)
      {
         if (loadedScene == null)
         {
            LogTools.warn("No HM3D scenes found. Convert one with scripts/hm3d_to_rdx.py.");
            setRealisticModel(RDXModelBuilder.createBox(1.0f, 1.0f, 1.0f, Color.RED).model);
            setBounds(1.0, 1.0, 0.0, 1.0);
            loadedScene = "";
         }
         return;
      }
      if (scene.equals(loadedScene))
         return;

      try
      {
         String directory = SCENES_DIRECTORY + scene + "/" + scene;
         JsonNode metadata = new ObjectMapper().readTree(Gdx.files.internal(directory + "_rdx.json").readString());
         getCollisionBoxes().clear();
         getCollisionBoxes().addAll(readBoxes(Gdx.files.internal(directory + "_collision_boxes.txt")));

         loadRealisticModel(directory + ".glb");
         JsonNode offset = metadata.get("visualOffset");
         getRealisticModelOffset().setToZero();
         getRealisticModelOffset().getTranslation().set(offset.get(0).asDouble(), offset.get(1).asDouble(), offset.get(2).asDouble());

         JsonNode min = metadata.get("boundsMin");
         JsonNode max = metadata.get("boundsMax");
         setBounds(max.get(0).asDouble() - min.get(0).asDouble(),
                   max.get(1).asDouble() - min.get(1).asDouble(),
                   min.get(2).asDouble(),
                   max.get(2).asDouble());
         loadedScene = scene;
         entrances.clear();
         for (JsonNode entrance : metadata.path("entrances"))
            entrances.add(new double[] {entrance.get("x").asDouble(), entrance.get("y").asDouble(), entrance.get("outwardYaw").asDouble(),
                                        entrance.get("width").asDouble()});
         entranceLabels = new String[entrances.size()];
         for (int i = 0; i < entrances.size(); i++)
            entranceLabels[i] = "%d: %.1f m wide at (%.1f, %.1f)".formatted(i, entrances.get(i)[3], entrances.get(i)[0], entrances.get(i)[1]);
         selectedEntrance.set(0);
         LogTools.info("Loaded HM3D scene {}: floors at {} m, {} collision boxes", scene, metadata.get("floorHeights"), getCollisionBoxes().size());
      }
      catch (IOException | RuntimeException exception)
      {
         LogTools.error("Could not load HM3D scene {}: {}", scene, exception.getMessage());
      }
   }

   private static List<TerrainBox> readBoxes(FileHandle file)
   {
      List<TerrainBox> boxes = new ArrayList<>();
      for (String line : file.readString().split("\\R"))
      {
         if (line.isBlank() || line.startsWith("#"))
            continue;
         String[] values = line.trim().split("\\s+");
         boxes.add(new TerrainBox(Double.parseDouble(values[0]),
                                  Double.parseDouble(values[1]),
                                  Double.parseDouble(values[2]),
                                  Double.parseDouble(values[3]),
                                  Double.parseDouble(values[4]),
                                  Double.parseDouble(values[5]),
                                  0));
      }
      return boxes;
   }

   /** Bounding box for picking and the footprint, shown as an outline since the scan is the visual. */
   private void setBounds(double sizeX, double sizeY, double minZ, double maxZ)
   {
      bounds[0] = sizeX;
      bounds[1] = sizeY;
      bounds[2] = minZ;
      bounds[3] = maxZ;
      boundingBox = new Box3D(sizeX, sizeY, maxZ - minZ);
      getCollisionShapeOffset().setToZero();
      getCollisionShapeOffset().getTranslation().setZ(0.5 * (minZ + maxZ));
      getBoundingSphere().setRadius(0.5 * Math.sqrt(sizeX * sizeX + sizeY * sizeY + (maxZ - minZ) * (maxZ - minZ)));
      Box3D outline = boundingBox;
      setCollisionModel(meshBuilder -> meshBuilder.addMultiLineBox(outline.getVertices(), 0.02, LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue())));
      setCollisionGeometryObject(boundingBox);
      updateRenderablesPoses();
   }

   @Override
   protected void buildMeshes(ArrayList<Consumer<RDXMultiColorMeshBuilder>> meshBuilders)
   {
      // The visual is the scan, loaded in rebuild
   }

   @Override
   protected double[] computeBounds()
   {
      return bounds.clone();
   }

   @Override
   protected boolean renderParameterWidgets()
   {
      if (availableScenes.length == 0)
      {
         ImGui.text("No HM3D scenes. Convert one with scripts/hm3d_to_rdx.py.");
         return false;
      }
      boolean sceneChanged = ImGui.combo(labels.get("Scene"), selectedScene, availableScenes);
      if (entrances.isEmpty())
         return sceneChanged;
      ImGui.combo(labels.get("Entrance"), selectedEntrance, entranceLabels);
      inputMeters("Distance from robot", entranceDistance, 0.5, 10.0);
      if (ImGui.button(labels.get("Put entrance in front of robot")))
         placeEntranceInFrontOfOrigin(entrances.get(selectedEntrance.get()), entranceDistance.get());
      return sceneChanged;
   }

   /**
    * The robot starts at the world origin facing +x. Turns and moves the building so the entrance is
    * straight ahead at the given distance, with its doorway facing the robot.
    */
   private void placeEntranceInFrontOfOrigin(double[] entrance, double distance)
   {
      double yaw = Math.PI - entrance[2];
      double cos = Math.cos(yaw);
      double sin = Math.sin(yaw);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setToYawOrientation(yaw);
      // Keep the height, which sits the scan just above the lab floor so the two do not flicker
      transform.getTranslation().set(distance - (cos * entrance[0] - sin * entrance[1]),
                                     -(sin * entrance[0] + cos * entrance[1]),
                                     getObjectTransform().getTranslationZ());
      setTransformToWorld(transform);
   }

   @Override
   public void saveParameters(ObjectNode objectNode)
   {
      if (sceneId() != null)
         objectNode.put("scene", sceneId());
   }

   @Override
   public void loadParameters(JsonNode objectNode)
   {
      int index = Arrays.asList(availableScenes).indexOf(objectNode.path("scene").asText(""));
      if (index >= 0)
         selectedScene.set(index);
      else if (objectNode.has("scene"))
         LogTools.warn("HM3D scene {} is not converted on this machine. Run scripts/hm3d_to_rdx.py.", objectNode.get("scene").asText());
      rebuild();
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      // updateRenderablesPoses keeps the bounding box posed in world
      if (boundingBox == null || boundingBox.intersectionWith(pickRay, firstIntersection, secondIntersection) == 0)
         return false;
      intersectionToPack.set(firstIntersection);
      return true;
   }
}
