package us.ihmc.simulationConstructionSetTools.util.environments.procedural;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainSurface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.ConvexPolytopeTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

/**
 * SCS environment built from procedural terrain ({@link ProceduralTerrainGeometry}), the same terrain the RDX
 * environment builder makes. Use it to build procedural terrain in code; {@code JSONEnvironment} uses it to
 * build environments saved from the UI.
 * <p>
 * Procedural ground has the procedural terrain cut out of it, so stairs down and pits open into it. The ground
 * is cut when the terrain is first requested, so terrain can be added before or after the ground.
 * </p>
 */
public class ProceduralTerrainEnvironment implements CommonAvatarEnvironmentInterface
{
   /** Thick enough that the physics engine can't push feet through it. */
   public static final double GROUND_THICKNESS = 0.5;
   public static final AppearanceDefinition DEFAULT_GROUND_APPEARANCE = YoAppearance.RGBColor(0.824, 0.706, 0.549);
   private static final AppearanceDefinition[] SHADES = {YoAppearance.LightGray(), YoAppearance.Gray(), YoAppearance.DarkGray()};

   private record Ground(RigidBodyTransform transform, double sizeX, double sizeY, AppearanceDefinition appearance)
   {
   }

   private final CombinedTerrainObject3D terrain;
   /** Footprints in world of the terrain that procedural ground is cut around */
   private final List<List<Point3D>> footprints = new ArrayList<>();
   private final List<Ground> grounds = new ArrayList<>();
   private boolean groundsAdded = false;

   public ProceduralTerrainEnvironment(String name)
   {
      terrain = new CombinedTerrainObject3D(name);
   }

   /**
    * Square stairs rising to a center platform; a negative {@code stepRise} builds inverted stairs down into a pit.
    * See {@link ProceduralTerrainGeometry#pyramidStairs}.
    */
   public void addPyramidStairs(RigidBodyTransformReadOnly transform, double baseSize, double platformSize, double stepRise, double stepRun)
   {
      addBoxes(transform, ProceduralTerrainGeometry.pyramidStairs(baseSize, platformSize, stepRise, stepRun));
      footprints.add(footprint(transform, baseSize, baseSize));
   }

   /**
    * A square hill trimmed to a flat platform; a negative {@code slopeDegrees} builds an inverted pyramid (a pit).
    * See {@link ProceduralTerrainGeometry#pyramidSlope}.
    */
   public void addPyramidSlope(RigidBodyTransformReadOnly transform, double baseSize, double platformSize, double slopeDegrees)
   {
      addSurfaces(transform, ProceduralTerrainGeometry.pyramidSlope(baseSize, platformSize, slopeDegrees));
      footprints.add(footprint(transform, baseSize, baseSize));
   }

   /** A grid of square tiles with random heights. See {@link ProceduralTerrainGeometry#unevenTiles}. */
   public void addUnevenTiles(RigidBodyTransformReadOnly transform, double tileSize, double maxHeightDelta, double gridSize, long seed)
   {
      addBoxes(transform, ProceduralTerrainGeometry.unevenTiles(tileSize, maxHeightDelta, gridSize, seed));
      double width = ProceduralTerrainGeometry.unevenTilesPerSide(tileSize, gridSize) * tileSize;
      footprints.add(footprint(transform, width, width));
   }

   /**
    * A bumpy surface of random heights that meets the ground at its edges.
    * See {@link ProceduralTerrainGeometry#randomHeightField}.
    */
   public void addRandomHeightField(RigidBodyTransformReadOnly transform, double cellSize, double maxHeightDelta, double gridSize, long seed)
   {
      addSurfaces(transform, ProceduralTerrainGeometry.randomHeightField(cellSize, maxHeightDelta, gridSize, seed));
      double width = ProceduralTerrainGeometry.randomHeightFieldCellsPerSide(cellSize, gridSize) * cellSize;
      footprints.add(footprint(transform, width, width));
   }

   public void addProceduralGround(RigidBodyTransformReadOnly transform, double sizeX, double sizeY)
   {
      addProceduralGround(transform, sizeX, sizeY, DEFAULT_GROUND_APPEARANCE);
   }

   /** A sizeX by sizeY ground, top at the transform's z, with all the procedural terrain cut out of it. */
   public void addProceduralGround(RigidBodyTransformReadOnly transform, double sizeX, double sizeY, AppearanceDefinition appearance)
   {
      grounds.add(new Ground(new RigidBodyTransform(transform), sizeX, sizeY, appearance));
   }

   public boolean hasProceduralGround()
   {
      return !grounds.isEmpty();
   }

   /** Adds any other terrain object, such as a box. The procedural ground is not cut around it. */
   public void addTerrainObject(TerrainObject3D terrainObject)
   {
      terrain.addTerrainObject(terrainObject);
   }

   public CombinedTerrainObject3D getCombinedTerrainObject3D()
   {
      return (CombinedTerrainObject3D) getTerrainObject3D();
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      if (!groundsAdded)
      {
         groundsAdded = true;
         for (Ground ground : grounds)
            addGround(ground);
      }
      return terrain;
   }

   private void addBoxes(RigidBodyTransformReadOnly objectTransform, List<TerrainBox> boxes)
   {
      for (TerrainBox box : boxes)
      {
         RigidBodyTransform boxTransform = new RigidBodyTransform(objectTransform);
         boxTransform.appendTranslation(box.x(), box.y(), box.z());
         terrain.addRotatableBox(boxTransform, box.sizeX(), box.sizeY(), box.sizeZ(), SHADES[box.shade()]);
      }
   }

   private void addSurfaces(RigidBodyTransformReadOnly objectTransform, List<TerrainSurface> surfaces)
   {
      for (TerrainSurface surface : surfaces)
         terrain.addTerrainObject(new ConvexPolytopeTerrainObject(toPolytope(objectTransform, surface), SHADES[surface.shade()]));
   }

   /** Each surface becomes the convex solid between it and its bottom. */
   private static ConvexPolytope3D toPolytope(RigidBodyTransformReadOnly objectTransform, TerrainSurface surface)
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
   private static List<Point3D> footprint(RigidBodyTransformReadOnly objectTransform, double sizeX, double sizeY)
   {
      List<Point3D> corners = List.of(new Point3D(-0.5 * sizeX, -0.5 * sizeY, 0.0),
                                      new Point3D(0.5 * sizeX, -0.5 * sizeY, 0.0),
                                      new Point3D(0.5 * sizeX, 0.5 * sizeY, 0.0),
                                      new Point3D(-0.5 * sizeX, 0.5 * sizeY, 0.0));
      for (Point3D corner : corners)
         objectTransform.transform(corner);
      return corners;
   }

   private void addGround(Ground ground)
   {
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

      for (List<Point2D> piece : ProceduralTerrainGeometry.groundPieces(ground.sizeX(), ground.sizeY(), holes))
      {
         double[] rectangle = ProceduralTerrainGeometry.asAxisAlignedRectangle(piece);
         if (rectangle != null)
         {
            // Boxes are MuJoCo primitives, cheaper and more robust than meshes
            RigidBodyTransform pieceTransform = new RigidBodyTransform(ground.transform());
            pieceTransform.appendTranslation(0.5 * (rectangle[0] + rectangle[2]), 0.5 * (rectangle[1] + rectangle[3]), -0.5 * GROUND_THICKNESS);
            terrain.addRotatableBox(pieceTransform, rectangle[2] - rectangle[0], rectangle[3] - rectangle[1], GROUND_THICKNESS, ground.appearance());
         }
         else
         {
            // Around terrain rotated relative to the ground
            List<Point3D> vertices = piece.stream().map(point -> new Point3D(point.getX(), point.getY(), 0.0)).toList();
            TerrainSurface surface = new TerrainSurface(vertices, -GROUND_THICKNESS, 0);
            terrain.addTerrainObject(new ConvexPolytopeTerrainObject(toPolytope(ground.transform(), surface), ground.appearance()));
         }
      }
   }
}
