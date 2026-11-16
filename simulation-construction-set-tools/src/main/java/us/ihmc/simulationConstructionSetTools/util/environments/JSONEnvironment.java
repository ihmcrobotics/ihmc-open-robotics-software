package us.ihmc.simulationConstructionSetTools.util.environments;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.log.LogTools;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainEnvironment;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

/**
 * SCS environment loaded from a JSON file saved by the RDX environment builder (RDXEnvironmentBuilder),
 * so environments built in the UI can be simulated without generating code.
 * <p>
 * Supported: pallets and cinder blocks (as boxes), and the procedural terrain built by
 * {@link ProceduralTerrainEnvironment}: pyramid stairs and uneven tiles (as boxes), hills and random
 * height fields (as convex solids under each flat surface), and procedural ground. Procedural ground has the other
 * procedural terrain cut out of it, so stairs down and pits open into it. Without procedural ground, a
 * 50 x 50 m ground box is added, as in {@link RDXEnvironments}. Other object types are skipped with a
 * warning.
 * </p>
 */
public class JSONEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final String REPOSITORY = "ihmc-open-robotics-software";
   private static final String RESOURCES_FOLDER = "ihmc-high-level-behaviors/src/libgdx/resources";
   private static final String ENVIRONMENTS_FOLDER = "environments";

   private record LoadedObject(String type, RigidBodyTransform transform, JsonNode node)
   {
   }

   private final ProceduralTerrainEnvironment terrain;

   /**
    * @param environmentFile a file name in ihmc-high-level-behaviors/src/libgdx/resources/environments,
    *                        e.g. "ProceduralTerrainExample.json", or a path to a JSON file.
    */
   public JSONEnvironment(String environmentFile)
   {
      Path path = findEnvironmentFile(environmentFile);
      terrain = new ProceduralTerrainEnvironment(path.getFileName().toString());

      ArrayList<LoadedObject> objects = new ArrayList<>();
      JSONFileTools.load(path, rootNode -> JSONTools.forEachArrayElement(rootNode, "objects", objectNode ->
      {
         RigidBodyTransform transform = new RigidBodyTransform(new Quaternion(objectNode.get("qx").asDouble(),
                                                                             objectNode.get("qy").asDouble(),
                                                                             objectNode.get("qz").asDouble(),
                                                                             objectNode.get("qs").asDouble()),
                                                              new Point3D(objectNode.get("x").asDouble(),
                                                                          objectNode.get("y").asDouble(),
                                                                          objectNode.get("z").asDouble()));
         objects.add(new LoadedObject(objectNode.get("type").asText(), transform, objectNode));
      }));

      for (LoadedObject object : objects)
      {
         JsonNode node = object.node();
         switch (object.type())
         {
            case "RDXPyramidStairsObject" -> terrain.addPyramidStairs(object.transform(),
                                                                       node.get("baseSize").asDouble(),
                                                                       node.get("platformSize").asDouble(),
                                                                       node.get("stepRise").asDouble(),
                                                                       node.get("stepRun").asDouble());
            case "RDXUnevenTilesObject" -> terrain.addUnevenTiles(object.transform(),
                                                                   node.get("tileSize").asDouble(),
                                                                   node.get("maxHeightDelta").asDouble(),
                                                                   node.get("gridSize").asDouble(),
                                                                   node.get("seed").asLong());
            case "RDXPyramidSlopeObject" -> terrain.addPyramidSlope(object.transform(),
                                                                     node.get("baseSize").asDouble(),
                                                                     node.get("platformSize").asDouble(),
                                                                     node.get("slopeDegrees").asDouble());
            case "RDXRandomHeightFieldObject" -> terrain.addRandomHeightField(object.transform(),
                                                                          node.get("cellSize").asDouble(),
                                                                          node.get("maxHeightDelta").asDouble(),
                                                                          node.get("gridSize").asDouble(),
                                                                          node.get("seed").asLong());
            case "RDXProceduralGroundObject" ->
            {
               AppearanceDefinition appearance = ProceduralTerrainEnvironment.DEFAULT_GROUND_APPEARANCE;
               JsonNode colorNode = node.path("color");
               if (colorNode.size() == 3)
                  appearance = YoAppearance.RGBColor(colorNode.get(0).asDouble(), colorNode.get(1).asDouble(), colorNode.get(2).asDouble());
               terrain.addProceduralGround(object.transform(), node.get("sizeX").asDouble(), node.get("sizeY").asDouble(), appearance);
            }
            case "RDXLabFloorObject" ->
            {
               // Replaced by the ground below
            }
            default ->
            {
               Vector3D size = getBoxSize(object.type());
               if (size != null)
                  terrain.addTerrainObject(new RotatableBoxTerrainObject(new Box3D(object.transform(), size.getX(), size.getY(), size.getZ()),
                                                                         YoAppearance.DarkGray()));
               else
                  LogTools.warn("{} is not supported in simulation yet, skipping it", object.type());
            }
         }
      }

      if (!terrain.hasProceduralGround())
      {
         Box3D ground = new Box3D(50.0, 50.0, 1.0);
         ground.getPosition().setZ(-0.5);
         terrain.addTerrainObject(new RotatableBoxTerrainObject(ground, new YoAppearanceTexture("Textures/gridGroundProfile.png")));
      }
   }

   private static Path findEnvironmentFile(String environmentFile)
   {
      Path path = Path.of(environmentFile);
      if (!Files.isRegularFile(path))
      {
         Path environmentsDirectory = findEnvironmentsDirectory();
         path = environmentsDirectory == null ? null : environmentsDirectory.resolve(environmentFile);
      }
      if (path == null || !Files.isRegularFile(path))
         throw new RuntimeException("Could not find environment file " + environmentFile + " in " + REPOSITORY + "/" + RESOURCES_FOLDER + "/" + ENVIRONMENTS_FOLDER);
      LogTools.info("Loading environment: {}", path);
      return path;
   }

   /**
    * Searches up from where this class was loaded, then up from the working directory, for a folder
    * containing ihmc-open-robotics-software. The repository is often a sibling of the working directory
    * (e.g. running from a robot repository), which a search of the working directory alone misses.
    */
   private static Path findEnvironmentsDirectory()
   {
      ArrayList<Path> startingPoints = new ArrayList<>();
      try
      {
         startingPoints.add(Path.of(JSONEnvironment.class.getProtectionDomain().getCodeSource().getLocation().toURI()));
      }
      catch (Exception e)
      {
         LogTools.debug("Could not get code source location: {}", e.getMessage());
      }
      startingPoints.add(Path.of("").toAbsolutePath());

      for (Path startingPoint : startingPoints)
      {
         for (Path directory = startingPoint; directory != null; directory = directory.getParent())
         {
            Path candidate = directory.getFileName() != null && directory.getFileName().toString().equals(REPOSITORY) ? directory : directory.resolve(REPOSITORY);
            Path environmentsDirectory = candidate.resolve(RESOURCES_FOLDER).resolve(ENVIRONMENTS_FOLDER);
            if (Files.isDirectory(environmentsDirectory))
               return environmentsDirectory;
         }
      }
      return null;
   }

   /**
    * @return the size of the box used to simulate a mesh object, with the object position at its center,
    *       or null if the object has no box equivalent.
    */
   public static Vector3D getBoxSize(String type)
   {
      return switch (type)
      {
         case "RDXPalletObject" -> new Vector3D(1.21, 1.013, 0.155);
         case "RDXSmallCinderBlockRoughed" -> new Vector3D(0.393, 0.192, 0.0884);
         case "RDXMediumCinderBlockRoughed" -> new Vector3D(0.393001, 0.188522, 0.141535);
         case "RDXLargeCinderBlockRoughed" -> new Vector3D(0.393, 0.19, 0.192);
         default -> null;
      };
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain.getTerrainObject3D();
   }
}
