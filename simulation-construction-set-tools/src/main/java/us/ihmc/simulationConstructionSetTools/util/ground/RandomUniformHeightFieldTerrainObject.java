package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.Random;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

/**
 * A random, bumpy heightfield surface. Modeled on IsaacLab's HfRandomUniformTerrain (isaac-sim/IsaacLab,
 * {@code terrains/height_field/hf_terrains.py}, {@code random_uniform_terrain}): a coarse grid of independently
 * random control-point heights, connected into a continuous surface. This implementation bilinearly interpolates
 * within each grid cell (both for the rendered mesh and for height queries), rather than IsaacLab's bicubic spline
 * upsampling &mdash; a simpler, close approximation of the same "smooth random bumps" look.
 * <p>
 * The footprint is {@code sizeX x sizeY}, centered on the local origin. A flat {@code borderWidth} apron is left
 * around the outside (so the terrain tiles cleanly against flat ground); the remaining
 * {@code (sizeX - 2*borderWidth) x (sizeY - 2*borderWidth)} region is divided into cells roughly
 * {@code controlPointSpacing} wide, each corner independently assigned a random height drawn uniformly from
 * {@code [noiseMin, noiseMax]} (seeded by {@code seed}, so the surface is reproducible). Following IsaacLab's own
 * split between {@code downsampled_scale} (where the randomness lives) and {@code horizontal_scale} (the fine
 * heightfield/mesh resolution it's upsampled to), the rendered mesh is tessellated at a separate, typically much
 * finer, {@code meshResolution}: each coarse cell is resampled from the same bilinear surface used for height
 * queries, so the extra tessellation reveals the true (non-planar) curvature of each cell rather than rendering it
 * as a single flat quad. A {@link RigidBodyTransformReadOnly} may be supplied to place and orient the patch
 * anywhere in the world.
 */
public class RandomUniformHeightFieldTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private static final double NORMAL_FINITE_DIFFERENCE_STEP = 1.0e-3;

   private final RigidBodyTransformReadOnly transform;
   private final double sizeX;
   private final double sizeY;
   private final double controlPointSpacing;
   private final int numCellsX;
   private final int numCellsY;
   private final double gridSizeX;
   private final double gridSizeY;
   /** [i][j] is the height at control point (i, j), for i in [0, numCellsX] and j in [0, numCellsY]. */
   private final double[][] heights;

   private final BoundingBox3D boundingBox = new BoundingBox3D();
   private final Graphics3DObject linkGraphics;

   private final Point3D queryPointBuffer = new Point3D();

   public RandomUniformHeightFieldTerrainObject(long seed, double sizeX, double sizeY, double noiseMin, double noiseMax, double controlPointSpacing,
                                                 double meshResolution, double borderWidth, AppearanceDefinition appearance)
   {
      this(new RigidBodyTransform(), seed, sizeX, sizeY, noiseMin, noiseMax, controlPointSpacing, meshResolution, borderWidth, appearance);
   }

   public RandomUniformHeightFieldTerrainObject(RigidBodyTransformReadOnly transform, long seed, double sizeX, double sizeY, double noiseMin,
                                                 double noiseMax, double controlPointSpacing, double meshResolution, double borderWidth)
   {
      this(transform, seed, sizeX, sizeY, noiseMin, noiseMax, controlPointSpacing, meshResolution, borderWidth, YoAppearance.StoneTexture());
   }

   public RandomUniformHeightFieldTerrainObject(RigidBodyTransformReadOnly transform, long seed, double sizeX, double sizeY, double noiseMin,
                                                 double noiseMax, double controlPointSpacing, double meshResolution, double borderWidth,
                                                 AppearanceDefinition appearance)
   {
      if (sizeX <= 0.0 || sizeY <= 0.0)
         throw new IllegalArgumentException("sizeX and sizeY must be positive.");
      if (controlPointSpacing <= 0.0)
         throw new IllegalArgumentException("controlPointSpacing must be positive.");
      if (meshResolution <= 0.0 || meshResolution > controlPointSpacing)
         throw new IllegalArgumentException("meshResolution must be positive and no greater than controlPointSpacing.");
      if (noiseMax < noiseMin)
         throw new IllegalArgumentException("noiseMax must be >= noiseMin.");
      if (borderWidth < 0.0 || 2.0 * borderWidth >= Math.min(sizeX, sizeY))
         throw new IllegalArgumentException("borderWidth must be non-negative and less than half the smaller of sizeX/sizeY.");

      this.transform = transform;
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.controlPointSpacing = controlPointSpacing;

      numCellsX = Math.max(1, (int) Math.round((sizeX - 2.0 * borderWidth) / controlPointSpacing));
      numCellsY = Math.max(1, (int) Math.round((sizeY - 2.0 * borderWidth) / controlPointSpacing));
      gridSizeX = numCellsX * controlPointSpacing;
      gridSizeY = numCellsY * controlPointSpacing;

      Random random = new Random(seed);
      heights = new double[numCellsX + 1][numCellsY + 1];
      for (int i = 0; i <= numCellsX; i++)
         for (int j = 0; j <= numCellsY; j++)
            heights[i][j] = noiseMin + random.nextDouble() * (noiseMax - noiseMin);

      boundingBox.setToNaN();
      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double z : new double[] {noiseMin, noiseMax})
            {
               Point3D corner = new Point3D(signX * sizeX / 2.0, signY * sizeY / 2.0, z);
               corner.applyTransform(transform);
               boundingBox.updateToIncludePoint(corner);
            }
         }
      }

      linkGraphics = new Graphics3DObject();
      addLinkGraphics(meshResolution, borderWidth, appearance);
   }

   private void addLinkGraphics(double meshResolution, double borderWidth, AppearanceDefinition appearance)
   {
      // Flat border apron, as four quads (matching BoxStackingTerrainTools.addFlatBorder's band layout).
      if (borderWidth > 0.0)
      {
         addWorldQuad(-sizeX / 2.0, gridSizeY / 2.0, sizeX / 2.0, sizeY / 2.0, 0.0, appearance);
         addWorldQuad(-sizeX / 2.0, -sizeY / 2.0, sizeX / 2.0, -gridSizeY / 2.0, 0.0, appearance);
         addWorldQuad(gridSizeX / 2.0, -gridSizeY / 2.0, sizeX / 2.0, gridSizeY / 2.0, 0.0, appearance);
         addWorldQuad(-sizeX / 2.0, -gridSizeY / 2.0, -gridSizeX / 2.0, gridSizeY / 2.0, 0.0, appearance);
      }

      // Resample the bilinear surface at meshResolution (typically much finer than controlPointSpacing), so each
      // noisy cell's true curvature is visible instead of being flattened into a single quad.
      int cellSubdivisions = Math.max(1, (int) Math.round(controlPointSpacing / meshResolution));
      int totalFineX = numCellsX * cellSubdivisions;
      int totalFineY = numCellsY * cellSubdivisions;
      double fineSpacingX = gridSizeX / totalFineX;
      double fineSpacingY = gridSizeY / totalFineY;

      for (int i = 0; i < totalFineX; i++)
      {
         double x0 = -gridSizeX / 2.0 + i * fineSpacingX;
         double x1 = x0 + fineSpacingX;

         for (int j = 0; j < totalFineY; j++)
         {
            double y0 = -gridSizeY / 2.0 + j * fineSpacingY;
            double y1 = y0 + fineSpacingY;

            Point3D[] quad = {toWorld(x0, y0, heightAtLocal(x0, y0)), toWorld(x1, y0, heightAtLocal(x1, y0)), toWorld(x1, y1, heightAtLocal(x1, y1)),
                              toWorld(x0, y1, heightAtLocal(x0, y1))};
            linkGraphics.addPolygon(quad, appearance);
         }
      }
   }

   /** Adds a single flat (constant-{@code z}) quad, in local coordinates, to the link graphics. */
   private void addWorldQuad(double x0, double y0, double x1, double y1, double z, AppearanceDefinition appearance)
   {
      Point3D[] quad = {toWorld(x0, y0, z), toWorld(x1, y0, z), toWorld(x1, y1, z), toWorld(x0, y1, z)};
      linkGraphics.addPolygon(quad, appearance);
   }

   private Point3D toWorld(double localX, double localY, double localZ)
   {
      Point3D point = new Point3D(localX, localY, localZ);
      point.applyTransform(transform);
      return point;
   }

   /** Bilinearly interpolated height at local (x, y); {@code 0.0} in the flat border, {@code NEGATIVE_INFINITY} outside the footprint. */
   private double heightAtLocal(double localX, double localY)
   {
      if (localX < -sizeX / 2.0 || localX > sizeX / 2.0 || localY < -sizeY / 2.0 || localY > sizeY / 2.0)
         return Double.NEGATIVE_INFINITY;

      if (localX < -gridSizeX / 2.0 || localX > gridSizeX / 2.0 || localY < -gridSizeY / 2.0 || localY > gridSizeY / 2.0)
         return 0.0;

      double fx = Math.min(Math.max((localX + gridSizeX / 2.0) / controlPointSpacing, 0.0), numCellsX);
      double fy = Math.min(Math.max((localY + gridSizeY / 2.0) / controlPointSpacing, 0.0), numCellsY);
      int i = Math.min((int) Math.floor(fx), numCellsX - 1);
      int j = Math.min((int) Math.floor(fy), numCellsY - 1);
      double u = fx - i;
      double v = fy - j;

      double h00 = heights[i][j];
      double h10 = heights[i + 1][j];
      double h01 = heights[i][j + 1];
      double h11 = heights[i + 1][j + 1];

      double h0 = h00 * (1.0 - u) + h10 * u;
      double h1 = h01 * (1.0 - u) + h11 * u;
      return h0 * (1.0 - v) + h1 * v;
   }

   private double heightAtWorld(double worldX, double worldY)
   {
      queryPointBuffer.set(worldX, worldY, 0.0);
      queryPointBuffer.applyInverseTransform(transform);
      double localHeight = heightAtLocal(queryPointBuffer.getX(), queryPointBuffer.getY());
      if (localHeight == Double.NEGATIVE_INFINITY)
         return Double.NEGATIVE_INFINITY;

      queryPointBuffer.setZ(localHeight);
      queryPointBuffer.applyTransform(transform);
      return queryPointBuffer.getZ();
   }

   private void computeWorldNormal(double worldX, double worldY, Vector3DBasics normalToPack)
   {
      double heightXMinus = heightAtWorld(worldX - NORMAL_FINITE_DIFFERENCE_STEP, worldY);
      double heightXPlus = heightAtWorld(worldX + NORMAL_FINITE_DIFFERENCE_STEP, worldY);
      double heightYMinus = heightAtWorld(worldX, worldY - NORMAL_FINITE_DIFFERENCE_STEP);
      double heightYPlus = heightAtWorld(worldX, worldY + NORMAL_FINITE_DIFFERENCE_STEP);

      double dhdx = (heightXPlus - heightXMinus) / (2.0 * NORMAL_FINITE_DIFFERENCE_STEP);
      double dhdy = (heightYPlus - heightYMinus) / (2.0 * NORMAL_FINITE_DIFFERENCE_STEP);

      normalToPack.set(-dhdx, -dhdy, 1.0);
      normalToPack.normalize();
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      return heightAtWorld(x, y);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      double heightAt = heightAtWorld(x, y);
      if (normalToPack != null && heightAt != Double.NEGATIVE_INFINITY)
         computeWorldNormal(x, y, normalToPack);
      return heightAt;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      double heightAt = heightAtWorld(x, y);

      if (intersectionToPack != null)
         intersectionToPack.set(x, y, heightAt);

      if (normalToPack != null && heightAt != Double.NEGATIVE_INFINITY)
         computeWorldNormal(x, y, normalToPack);

      return z < heightAt;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
