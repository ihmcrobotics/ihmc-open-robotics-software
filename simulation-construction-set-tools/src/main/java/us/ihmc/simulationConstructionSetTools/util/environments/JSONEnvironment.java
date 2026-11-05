package us.ihmc.simulationConstructionSetTools.util.environments;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.log.LogTools;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainSurface;
import us.ihmc.simulationConstructionSetTools.util.ground.ConvexPolytopeTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * SCS environment loaded from a JSON file saved by the RDX environment builder (RDXEnvironmentBuilder),
 * so environments built in the UI can be simulated without generating code.
 * <p>
 * Supported: pallets and cinder blocks (as boxes), pyramid stairs and uneven tiles (as boxes), hills
 * (as convex solids under each flat surface), and procedural ground. Procedural ground has the other
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
   /** Thick enough that the physics engine can't push feet through it. */
   private static final double GROUND_THICKNESS = 0.5;
   private static final AppearanceDefinition[] SHADES = {YoAppearance.LightGray(), YoAppearance.Gray(), YoAppearance.DarkGray()};

   private record LoadedObject(String type, RigidBodyTransform transform, JsonNode node)
   {
   }

   private final CombinedTerrainObject3D terrain;

   /**
    * @param environmentFile a file name in ihmc-high-level-behaviors/src/libgdx/resources/environments,
    *                        e.g. "ProceduralTerrainExample.json", or a path to a JSON file.
    */
   public JSONEnvironment(String environmentFile)
   {
      Path path = findEnvironmentFile(environmentFile);
      terrain = new CombinedTerrainObject3D(path.getFileName().toString());

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

      // Footprints in world of the terrain that procedural ground is cut around
      ArrayList<List<Point3D>> footprints = new ArrayList<>();
      ArrayList<LoadedObject> grounds = new ArrayList<>();

      for (LoadedObject object : objects)
      {
         JsonNode node = object.node();
         switch (object.type())
         {
            case "RDXPyramidStairsObject" ->
            {
               double baseSize = node.get("baseSize").asDouble();
               addBoxes(object.transform(),
                        ProceduralTerrainGeometry.pyramidStairs(baseSize,
                                                                node.get("platformSize").asDouble(),
                                                                node.get("stepRise").asDouble(),
                                                                node.get("stepRun").asDouble()));
               footprints.add(footprint(object.transform(), baseSize, baseSize));
            }
            case "RDXUnevenTilesObject" ->
            {
               double tileSize = node.get("tileSize").asDouble();
               double gridSize = node.get("gridSize").asDouble();
               addBoxes(object.transform(),
                        ProceduralTerrainGeometry.unevenTiles(tileSize, node.get("maxHeightDelta").asDouble(), gridSize, node.get("seed").asLong()));
               double width = ProceduralTerrainGeometry.unevenTilesPerSide(tileSize, gridSize) * tileSize;
               footprints.add(footprint(object.transform(), width, width));
            }
            case "RDXPyramidSlopeObject" ->
            {
               double baseSize = node.get("baseSize").asDouble();
               addSurfaces(object.transform(),
                           ProceduralTerrainGeometry.pyramidSlope(baseSize, node.get("platformSize").asDouble(), node.get("slopeDegrees").asDouble()));
               footprints.add(footprint(object.transform(), baseSize, baseSize));
            }
            case "RDXProceduralGroundObject" -> grounds.add(object);
            case "RDXLabFloorObject" ->
            {
               // Replaced by the ground below
            }
            default ->
            {
               Vector3D size = getBoxSize(object.type());
               if (size != null)
                  terrain.addRotatableBox(new Box3D(object.transform(), size.getX(), size.getY(), size.getZ()), YoAppearance.DarkGray());
               else
                  LogTools.warn("{} is not supported in simulation yet, skipping it", object.type());
            }
         }
      }

      if (grounds.isEmpty())
      {
         Box3D ground = new Box3D(50.0, 50.0, 1.0);
         ground.getPosition().setZ(-0.5);
         terrain.addTerrainObject(new RotatableBoxTerrainObject(ground, new YoAppearanceTexture("Textures/gridGroundProfile.png")));
      }
      for (LoadedObject ground : grounds)
      {
         addProceduralGround(ground, footprints);
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

   private void addBoxes(RigidBodyTransform objectTransform, List<TerrainBox> boxes)
   {
      for (TerrainBox box : boxes)
      {
         RigidBodyTransform boxTransform = new RigidBodyTransform(objectTransform);
         boxTransform.appendTranslation(box.x(), box.y(), box.z());
         terrain.addRotatableBox(boxTransform, box.sizeX(), box.sizeY(), box.sizeZ(), SHADES[box.shade()]);
      }
   }

   /** Each surface becomes the convex solid between it and its bottom. */
   private void addSurfaces(RigidBodyTransform objectTransform, List<TerrainSurface> surfaces)
   {
      for (TerrainSurface surface : surfaces)
         terrain.addTerrainObject(new ConvexPolytopeTerrainObject(toPolytope(objectTransform, surface), SHADES[surface.shade()]));
   }

   private static ConvexPolytope3D toPolytope(RigidBodyTransform objectTransform, TerrainSurface surface)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      for (Point3D vertex : surface.vertices())
      {
         polytope.addVertex(vertex);
         polytope.addVertex(new Point3D(vertex.getX(), vertex.getY(), surface.bottomZ()));
      }
      polytope.applyTransform(objectTransform);
      return polytope;
   }

   /** Corners of a sizeX by sizeY rectangle centered on the object, counter-clockwise, in world. */
   private static List<Point3D> footprint(RigidBodyTransform objectTransform, double sizeX, double sizeY)
   {
      List<Point3D> corners = List.of(new Point3D(-0.5 * sizeX, -0.5 * sizeY, 0.0),
                                      new Point3D(0.5 * sizeX, -0.5 * sizeY, 0.0),
                                      new Point3D(0.5 * sizeX, 0.5 * sizeY, 0.0),
                                      new Point3D(-0.5 * sizeX, 0.5 * sizeY, 0.0));
      for (Point3D corner : corners)
         objectTransform.transform(corner);
      return corners;
   }

   private void addProceduralGround(LoadedObject ground, List<List<Point3D>> footprints)
   {
      JsonNode node = ground.node();
      AppearanceDefinition appearance = YoAppearance.RGBColor(0.824, 0.706, 0.549);
      JsonNode colorNode = node.path("color");
      if (colorNode.size() == 3)
         appearance = YoAppearance.RGBColor(colorNode.get(0).asDouble(), colorNode.get(1).asDouble(), colorNode.get(2).asDouble());

      ArrayList<List<Point2D>> holes = new ArrayList<>();
      for (List<Point3D> footprint : footprints)
      {
         ArrayList<Point2D> hole = new ArrayList<>();
         for (Point3D cornerInWorld : footprint)
         {
            Point3D corner = new Point3D(cornerInWorld);
            ground.transform().inverseTransform(corner);
            hole.add(new Point2D(corner.getX(), corner.getY()));
         }
         holes.add(hole);
      }

      for (List<Point2D> piece : ProceduralTerrainGeometry.groundPieces(node.get("sizeX").asDouble(), node.get("sizeY").asDouble(), holes))
      {
         double[] rectangle = ProceduralTerrainGeometry.asAxisAlignedRectangle(piece);
         if (rectangle != null)
         {
            // Boxes are MuJoCo primitives, cheaper and more robust than meshes
            RigidBodyTransform pieceTransform = new RigidBodyTransform(ground.transform());
            pieceTransform.appendTranslation(0.5 * (rectangle[0] + rectangle[2]), 0.5 * (rectangle[1] + rectangle[3]), -0.5 * GROUND_THICKNESS);
            terrain.addRotatableBox(pieceTransform, rectangle[2] - rectangle[0], rectangle[3] - rectangle[1], GROUND_THICKNESS, appearance);
         }
         else
         {
            // Around terrain rotated relative to the ground
            List<Point3D> vertices = piece.stream().map(point -> new Point3D(point.getX(), point.getY(), 0.0)).toList();
            TerrainSurface surface = new TerrainSurface(vertices, -GROUND_THICKNESS, 0);
            terrain.addTerrainObject(new ConvexPolytopeTerrainObject(toPolytope(ground.transform(), surface), appearance));
         }
      }
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
      return terrain;
   }
}
