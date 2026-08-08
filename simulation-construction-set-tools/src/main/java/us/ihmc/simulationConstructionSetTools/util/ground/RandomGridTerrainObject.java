package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

/**
 * A square grid of cells, each independently shifted up or down by a random amount, trimming to a flat platform at
 * the center. Modeled on IsaacLab's MeshRandomGridTerrain (isaac-sim/IsaacLab,
 * {@code terrains/trimesh/mesh_terrains.py}, {@code random_grid_terrain}).
 * <p>
 * The terrain footprint is {@code size x size} (must be square, like IsaacLab's), centered on the local origin,
 * divided into {@code gridWidth}-square cells. Each cell's top surface is independently offset by a value drawn
 * uniformly from {@code [-gridHeight, gridHeight]} (seeded by {@code seed}, so the layout is reproducible); the
 * bottom of every cell stays level. A flat platform of {@code platformWidth} sits at the center, its top flush with
 * the highest possible cell (matching IsaacLab). If {@code holes} is {@code true}, only cells within a
 * cross-shaped column extending from the platform along both axes are generated (no border), approximating
 * IsaacLab's {@code cfg.holes} masking.
 */
public class RandomGridTerrainObject extends CombinedTerrainObject3D
{
   private static final double TERRAIN_HEIGHT = 1.0;

   public RandomGridTerrainObject(String name, long seed, double size, double gridWidth, double gridHeight, double platformWidth, boolean holes,
                                   AppearanceDefinition appearance)
   {
      this(name, new RigidBodyTransform(), seed, size, gridWidth, gridHeight, platformWidth, holes, appearance);
   }

   public RandomGridTerrainObject(String name, RigidBodyTransformReadOnly transform, long seed, double size, double gridWidth, double gridHeight,
                                   double platformWidth, boolean holes)
   {
      this(name, transform, seed, size, gridWidth, gridHeight, platformWidth, holes, YoAppearance.StoneTexture());
   }

   public RandomGridTerrainObject(String name, RigidBodyTransformReadOnly transform, long seed, double size, double gridWidth, double gridHeight,
                                   double platformWidth, boolean holes, AppearanceDefinition appearance)
   {
      super(name);

      if (size <= 0.0)
         throw new IllegalArgumentException("size must be positive.");
      if (gridWidth <= 0.0)
         throw new IllegalArgumentException("gridWidth must be positive.");
      if (gridHeight < 0.0)
         throw new IllegalArgumentException("gridHeight must be non-negative.");
      if (platformWidth <= 0.0)
         throw new IllegalArgumentException("platformWidth must be positive.");

      int numBoxes = (int) (size / gridWidth);
      double borderWidth = size - numBoxes * gridWidth;
      if (borderWidth <= 0.0)
         throw new IllegalArgumentException("gridWidth is too large for the given size (border would be non-positive); reduce gridWidth.");

      BoxStackingTerrainTools.addFlatBorder(this,
                                             transform,
                                             size,
                                             size,
                                             size - borderWidth,
                                             size - borderWidth,
                                             TERRAIN_HEIGHT,
                                             -TERRAIN_HEIGHT / 2.0,
                                             appearance);

      Random random = new Random(seed);
      double gridOrigin = -size / 2.0 + borderWidth / 2.0;
      double halfHolesBand = (borderWidth + platformWidth) / 2.0;

      for (int i = 0; i < numBoxes; i++)
      {
         for (int j = 0; j < numBoxes; j++)
         {
            double cellCenterX = gridOrigin + (i + 0.5) * gridWidth;
            double cellCenterY = gridOrigin + (j + 0.5) * gridWidth;

            if (holes)
            {
               boolean verticalStrip = Math.abs(cellCenterX) <= halfHolesBand;
               boolean horizontalStrip = Math.abs(cellCenterY) <= halfHolesBand;
               if (!verticalStrip && !horizontalStrip)
                  continue;
            }

            // Only the top of the cell moves: the bottom stays at -TERRAIN_HEIGHT/2 for every cell, so neighboring
            // cells share a level floor even though their tops are independently jittered.
            double heightJitter = (random.nextDouble() * 2.0 - 1.0) * gridHeight;
            double cellHeight = TERRAIN_HEIGHT + heightJitter;
            double cellZ = -TERRAIN_HEIGHT / 2.0 + cellHeight / 2.0;
            BoxStackingTerrainTools.addLocalBox(this, transform, cellCenterX, cellCenterY, cellZ, gridWidth, gridWidth, cellHeight, appearance);
         }
      }

      double platformHeight = TERRAIN_HEIGHT + gridHeight;
      double platformZ = -TERRAIN_HEIGHT / 2.0 + gridHeight / 2.0;
      BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, 0.0, platformZ, platformWidth, platformWidth, platformHeight, appearance);
   }
}
