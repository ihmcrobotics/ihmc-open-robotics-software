package us.ihmc.simulationConstructionSetTools.util.environments.procedural;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Geometry for procedural terrain, shared by the RDX environment builder and the SCS simulation
 * environment so both build the same terrain from the same JSON parameters.
 * <p>
 * Modeled after the IsaacLab terrain generators (isaaclab.terrains). Terrain is centered on its origin,
 * with z = 0 at the level of the surrounding ground.
 * </p>
 */
public class ProceduralTerrainGeometry
{
   /** Solid material added under the lowest surface so the terrain is never paper thin. */
   public static final double BASE_THICKNESS = 0.05;
   public static final int MAX_TILES_PER_SIDE = 150;
   private static final double EPSILON = 1e-9;

   /**
    * An axis-aligned box in the terrain frame. {@code shade} is 0 for light, 1 for medium, 2 for dark,
    * so each renderer can pick its own colors.
    */
   public record TerrainBox(double sizeX, double sizeY, double sizeZ, double x, double y, double z, int shade)
   {
   }

   /** A box spanning bottomZ to topZ, centered at (x, y). */
   public static TerrainBox column(double sizeX, double sizeY, double x, double y, double bottomZ, double topZ, int shade)
   {
      return new TerrainBox(sizeX, sizeY, topZ - bottomZ, x, y, 0.5 * (bottomZ + topZ), shade);
   }

   // Pyramid stairs, after IsaacLab's MeshPyramidStairsTerrainCfg / MeshInvertedPyramidStairsTerrainCfg

   /**
    * IsaacLab adds one extra step, which can shrink the platform below platform_width; here the number of
    * steps is chosen so the center platform is at least {@code platformSize} wide.
    */
   public static int pyramidStairsNumberOfSteps(double baseSize, double platformSize, double stepRun)
   {
      return (int) Math.max(0, Math.floor((baseSize - platformSize) / (2.0 * stepRun) + EPSILON));
   }

   public static double pyramidStairsPlatformHeight(double baseSize, double platformSize, double stepRise, double stepRun)
   {
      return (pyramidStairsNumberOfSteps(baseSize, platformSize, stepRun) + 1) * stepRise;
   }

   public static double pyramidStairsBottomZ(double baseSize, double platformSize, double stepRise, double stepRun)
   {
      return Math.min(0.0, pyramidStairsPlatformHeight(baseSize, platformSize, stepRise, stepRun)) - BASE_THICKNESS;
   }

   /**
    * Each step is a square ring of width {@code stepRun}; ring k (counting in from the outer edge) has its
    * top at (k + 1) * stepRise and the center platform is one rise above the innermost ring. A positive rise
    * builds stairs up, a negative rise builds stairs down into a pit below the surrounding ground.
    */
   public static List<TerrainBox> pyramidStairs(double baseSize, double platformSize, double stepRise, double stepRun)
   {
      double bottomZ = pyramidStairsBottomZ(baseSize, platformSize, stepRise, stepRun);
      int numberOfSteps = pyramidStairsNumberOfSteps(baseSize, platformSize, stepRun);

      ArrayList<TerrainBox> boxes = new ArrayList<>();
      for (int k = 0; k < numberOfSteps; k++)
      {
         int shade = k % 2;
         double ringSize = baseSize - 2.0 * k * stepRun;
         double topZ = (k + 1) * stepRise;
         double offset = 0.5 * ringSize - 0.5 * stepRun;
         // Front and back strips span the full ring width; left and right fit between them
         boxes.add(column(ringSize, stepRun, 0.0, offset, bottomZ, topZ, shade));
         boxes.add(column(ringSize, stepRun, 0.0, -offset, bottomZ, topZ, shade));
         boxes.add(column(stepRun, ringSize - 2.0 * stepRun, offset, 0.0, bottomZ, topZ, shade));
         boxes.add(column(stepRun, ringSize - 2.0 * stepRun, -offset, 0.0, bottomZ, topZ, shade));
      }
      double centerSize = baseSize - 2.0 * numberOfSteps * stepRun;
      double platformHeight = pyramidStairsPlatformHeight(baseSize, platformSize, stepRise, stepRun);
      boxes.add(column(centerSize, centerSize, 0.0, 0.0, bottomZ, platformHeight, 2));
      return boxes;
   }

   /**
    * A flat, convex surface in the terrain frame with vertices counter-clockwise seen from above, filled
    * straight down to {@code bottomZ}. The filled volume is a convex solid, so a set of these can model any
    * terrain made of flat surfaces. {@code shade} is as in {@link TerrainBox}.
    */
   public record TerrainSurface(List<Point3D> vertices, double bottomZ, int shade)
   {
   }

   // Pyramid slope (hill), after IsaacLab's HfPyramidSlopedTerrainCfg / HfInvertedPyramidSlopedTerrainCfg

   /** tan(slope) * (baseSize - platformSize) / 2 */
   public static double pyramidSlopePlatformHeight(double baseSize, double platformSize, double slopeDegrees)
   {
      return Math.tan(Math.toRadians(slopeDegrees)) * 0.5 * (baseSize - platformSize);
   }

   public static double pyramidSlopeBottomZ(double baseSize, double platformSize, double slopeDegrees)
   {
      return Math.min(0.0, pyramidSlopePlatformHeight(baseSize, platformSize, slopeDegrees)) - BASE_THICKNESS;
   }

   /**
    * A square pyramid that trims to a flat platform at the center: the platform plus four planar faces
    * rising from the outer edge at ground level to the platform edge at the given slope angle. A positive
    * angle builds a hill, a negative angle builds a pit below the surrounding ground.
    * <p>
    * IsaacLab builds this as a height field whose slope is a rise/run ratio and whose faces are the product
    * of two ramps; here the faces are true planes so the slope is constant across each face.
    * </p>
    */
   public static List<TerrainSurface> pyramidSlope(double baseSize, double platformSize, double slopeDegrees)
   {
      double outer = 0.5 * baseSize;
      double inner = 0.5 * platformSize;
      double platformHeight = pyramidSlopePlatformHeight(baseSize, platformSize, slopeDegrees);
      double bottomZ = pyramidSlopeBottomZ(baseSize, platformSize, slopeDegrees);
      boolean hasPlatform = platformSize > 1e-3;

      // Counter-clockwise corners seen from above
      double[][] directions = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
      Point3D[] outerCorners = new Point3D[4];
      Point3D[] platformCorners = new Point3D[4];
      for (int i = 0; i < 4; i++)
      {
         outerCorners[i] = new Point3D(directions[i][0] * outer, directions[i][1] * outer, 0.0);
         platformCorners[i] = new Point3D(directions[i][0] * inner, directions[i][1] * inner, platformHeight);
      }

      ArrayList<TerrainSurface> surfaces = new ArrayList<>();
      if (hasPlatform)
         surfaces.add(new TerrainSurface(List.of(platformCorners), bottomZ, 2));
      for (int i = 0; i < 4; i++)
      {
         int next = (i + 1) % 4;
         List<Point3D> face = hasPlatform ? List.of(outerCorners[i], outerCorners[next], platformCorners[next], platformCorners[i])
                                          : List.of(outerCorners[i], outerCorners[next], platformCorners[i]);
         surfaces.add(new TerrainSurface(face, bottomZ, i % 2));
      }
      return surfaces;
   }

   // Uneven tiles, after IsaacLab's MeshRandomGridTerrainCfg

   public static int unevenTilesPerSide(double tileSize, double gridSize)
   {
      return (int) Math.min(MAX_TILES_PER_SIDE, Math.max(1, Math.floor(gridSize / tileSize + EPSILON)));
   }

   /**
    * Each tile top is drawn uniformly from [0, maxHeightDelta], so no two tiles differ by more than
    * maxHeightDelta. (IsaacLab draws from [-grid_height, grid_height], so its equivalent delta is
    * 2 * grid_height.) The same seed always gives the same heights.
    */
   public static List<TerrainBox> unevenTiles(double tileSize, double maxHeightDelta, double gridSize, long seed)
   {
      int tilesPerSide = unevenTilesPerSide(tileSize, gridSize);
      double halfWidth = 0.5 * tilesPerSide * tileSize;
      Random random = new Random(seed);

      ArrayList<TerrainBox> boxes = new ArrayList<>();
      for (int i = 0; i < tilesPerSide; i++)
      {
         for (int j = 0; j < tilesPerSide; j++)
         {
            double x = -halfWidth + (i + 0.5) * tileSize;
            double y = -halfWidth + (j + 0.5) * tileSize;
            double topZ = random.nextDouble() * maxHeightDelta;
            boxes.add(column(tileSize, tileSize, x, y, -BASE_THICKNESS, topZ, (i + j) % 2));
         }
      }
      return boxes;
   }

   // Ground with holes

   /**
    * @return convex, counter-clockwise pieces covering the sizeX by sizeY rectangle centered on the origin,
    *       minus the given convex, counter-clockwise holes.
    */
   public static List<List<Point2D>> groundPieces(double sizeX, double sizeY, List<? extends List<Point2D>> holes)
   {
      double halfX = 0.5 * sizeX;
      double halfY = 0.5 * sizeY;
      List<List<Point2D>> pieces = new ArrayList<>();
      pieces.add(List.of(new Point2D(-halfX, -halfY), new Point2D(halfX, -halfY), new Point2D(halfX, halfY), new Point2D(-halfX, halfY)));
      for (List<Point2D> hole : holes)
      {
         List<List<Point2D>> remainingPieces = new ArrayList<>();
         for (List<Point2D> piece : pieces)
            remainingPieces.addAll(subtract(piece, hole));
         pieces = remainingPieces;
      }
      return pieces;
   }

   /**
    * @return convex pieces that together cover {@code polygon} minus {@code hole}; both must be convex
    *       and counter-clockwise. Each hole edge splits off the part of the polygon outside it.
    */
   public static List<List<Point2D>> subtract(List<Point2D> polygon, List<Point2D> hole)
   {
      List<List<Point2D>> pieces = new ArrayList<>();
      List<Point2D> remaining = polygon;
      for (int i = 0; i < hole.size() && !remaining.isEmpty(); i++)
      {
         Point2D start = hole.get(i);
         Point2D end = hole.get((i + 1) % hole.size());
         List<Point2D> outside = clipToHalfPlane(remaining, start, end, false);
         if (area(outside) > EPSILON)
            pieces.add(outside);
         remaining = clipToHalfPlane(remaining, start, end, true);
      }
      return pieces;
   }

   /** Sutherland-Hodgman clip of a convex polygon to the left (inside) or right of the line start to end. */
   private static List<Point2D> clipToHalfPlane(List<Point2D> polygon, Point2D start, Point2D end, boolean keepLeft)
   {
      ArrayList<Point2D> clipped = new ArrayList<>();
      for (int i = 0; i < polygon.size(); i++)
      {
         Point2D current = polygon.get(i);
         Point2D next = polygon.get((i + 1) % polygon.size());
         double currentSide = side(start, end, current) * (keepLeft ? 1.0 : -1.0);
         double nextSide = side(start, end, next) * (keepLeft ? 1.0 : -1.0);
         if (currentSide >= 0.0)
            clipped.add(current);
         if ((currentSide > 0.0 && nextSide < 0.0) || (currentSide < 0.0 && nextSide > 0.0))
         {
            double alpha = currentSide / (currentSide - nextSide);
            clipped.add(new Point2D(current.getX() + alpha * (next.getX() - current.getX()), current.getY() + alpha * (next.getY() - current.getY())));
         }
      }
      return clipped;
   }

   private static double side(Point2D start, Point2D end, Point2D point)
   {
      return (end.getX() - start.getX()) * (point.getY() - start.getY()) - (end.getY() - start.getY()) * (point.getX() - start.getX());
   }

   public static double area(List<Point2D> polygon)
   {
      double twiceArea = 0.0;
      for (int i = 0; i < polygon.size(); i++)
      {
         Point2D current = polygon.get(i);
         Point2D next = polygon.get((i + 1) % polygon.size());
         twiceArea += current.getX() * next.getY() - next.getX() * current.getY();
      }
      return 0.5 * twiceArea;
   }

   public static boolean isInside(List<Point2D> convexPolygon, double x, double y)
   {
      Point2D point = new Point2D(x, y);
      for (int i = 0; i < convexPolygon.size(); i++)
      {
         if (side(convexPolygon.get(i), convexPolygon.get((i + 1) % convexPolygon.size()), point) < 0.0)
            return false;
      }
      return true;
   }

   /**
    * @return { minX, minY, maxX, maxY } if the polygon is an axis-aligned rectangle (so it can be a box),
    *       otherwise null.
    */
   public static double[] asAxisAlignedRectangle(List<Point2D> polygon)
   {
      double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
      for (Point2D point : polygon)
      {
         minX = Math.min(minX, point.getX());
         minY = Math.min(minY, point.getY());
         maxX = Math.max(maxX, point.getX());
         maxY = Math.max(maxY, point.getY());
      }
      double tolerance = 1e-6;
      double rectangleArea = (maxX - minX) * (maxY - minY);
      return Math.abs(area(polygon) - rectangleArea) < tolerance * Math.max(1.0, rectangleArea) ? new double[] {minX, minY, maxX, maxY} : null;
   }
}
