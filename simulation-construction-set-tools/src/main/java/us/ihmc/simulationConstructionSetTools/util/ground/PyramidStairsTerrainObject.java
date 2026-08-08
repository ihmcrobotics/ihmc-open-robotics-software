package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

/**
 * A concentric square stair pattern that ascends from the ground toward a flat platform at the center, built from
 * stacked boxes. Modeled on IsaacLab's MeshPyramidStairsTerrain (isaac-sim/IsaacLab,
 * {@code terrains/trimesh/mesh_terrains.py}, {@code pyramid_stairs_terrain}).
 * <p>
 * The terrain footprint is {@code sizeX x sizeY}, centered on the local origin. Steps ring inward from the edges
 * (minus an optional flat {@code borderWidth} apron) toward a flat {@code platformWidth} platform at the center,
 * each ring {@code stepWidth} wide and rising by {@code stepHeight}. If {@code holes} is {@code true}, only a
 * {@code platformWidth}-wide cross-shaped column of steps is generated (no border), matching IsaacLab's
 * {@code cfg.holes} behavior.
 */
public class PyramidStairsTerrainObject extends CombinedTerrainObject3D
{
   public PyramidStairsTerrainObject(String name, double sizeX, double sizeY, double stepHeight, double stepWidth, double platformWidth,
                                      double borderWidth, boolean holes, AppearanceDefinition appearance)
   {
      this(name, new RigidBodyTransform(), sizeX, sizeY, stepHeight, stepWidth, platformWidth, borderWidth, holes, appearance);
   }

   public PyramidStairsTerrainObject(String name, RigidBodyTransformReadOnly transform, double sizeX, double sizeY, double stepHeight, double stepWidth,
                                      double platformWidth, double borderWidth, boolean holes)
   {
      this(name, transform, sizeX, sizeY, stepHeight, stepWidth, platformWidth, borderWidth, holes, YoAppearance.StoneTexture());
   }

   public PyramidStairsTerrainObject(String name, RigidBodyTransformReadOnly transform, double sizeX, double sizeY, double stepHeight, double stepWidth,
                                      double platformWidth, double borderWidth, boolean holes, AppearanceDefinition appearance)
   {
      super(name);

      if (sizeX <= 0.0 || sizeY <= 0.0)
         throw new IllegalArgumentException("sizeX and sizeY must be positive.");
      if (stepHeight <= 0.0)
         throw new IllegalArgumentException("stepHeight must be positive.");
      if (stepWidth <= 0.0)
         throw new IllegalArgumentException("stepWidth must be positive.");
      if (platformWidth <= 0.0)
         throw new IllegalArgumentException("platformWidth must be positive.");
      if (borderWidth < 0.0)
         throw new IllegalArgumentException("borderWidth must be non-negative.");

      int numStepsX = (int) Math.floor((sizeX - 2.0 * borderWidth - platformWidth) / (2.0 * stepWidth)) + 1;
      int numStepsY = (int) Math.floor((sizeY - 2.0 * borderWidth - platformWidth) / (2.0 * stepWidth)) + 1;
      int numSteps = Math.min(numStepsX, numStepsY);
      if (numSteps < 0)
         throw new IllegalArgumentException("Terrain is too small for the given platformWidth/borderWidth/stepWidth.");

      double terrainSizeX = sizeX - 2.0 * borderWidth;
      double terrainSizeY = sizeY - 2.0 * borderWidth;

      if (borderWidth > 0.0 && !holes)
         BoxStackingTerrainTools.addFlatBorder(this, transform, sizeX, sizeY, terrainSizeX, terrainSizeY, stepHeight, -stepHeight / 2.0, appearance);

      for (int k = 0; k < numSteps; k++)
      {
         double boxSizeX = holes ? platformWidth : terrainSizeX - 2.0 * k * stepWidth;
         double boxSizeY = holes ? platformWidth : terrainSizeY - 2.0 * k * stepWidth;

         double boxZ = k * stepHeight / 2.0;
         double boxOffset = (k + 0.5) * stepWidth;
         double boxHeight = (k + 2) * stepHeight;

         // Top/bottom bands run the full box width in X; left/right bands fill the remaining inner strip in Y.
         BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, terrainSizeY / 2.0 - boxOffset, boxZ, boxSizeX, stepWidth, boxHeight, appearance);
         BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, -terrainSizeY / 2.0 + boxOffset, boxZ, boxSizeX, stepWidth, boxHeight, appearance);

         double sideSizeY = holes ? boxSizeY : boxSizeY - 2.0 * stepWidth;
         BoxStackingTerrainTools.addLocalBox(this, transform, terrainSizeX / 2.0 - boxOffset, 0.0, boxZ, stepWidth, sideSizeY, boxHeight, appearance);
         BoxStackingTerrainTools.addLocalBox(this, transform, -terrainSizeX / 2.0 + boxOffset, 0.0, boxZ, stepWidth, sideSizeY, boxHeight, appearance);
      }

      double middleSizeX = terrainSizeX - 2.0 * numSteps * stepWidth;
      double middleSizeY = terrainSizeY - 2.0 * numSteps * stepWidth;
      double middleHeight = (numSteps + 2) * stepHeight;
      double middleZ = numSteps * stepHeight / 2.0;
      BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, 0.0, middleZ, middleSizeX, middleSizeY, middleHeight, appearance);
   }
}
