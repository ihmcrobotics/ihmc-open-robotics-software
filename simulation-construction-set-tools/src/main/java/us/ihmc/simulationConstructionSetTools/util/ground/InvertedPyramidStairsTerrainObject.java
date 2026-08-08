package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

/**
 * A concentric square stair pattern that descends from the ground into a pit, trimming to a flat platform at the
 * bottom center. Modeled on IsaacLab's MeshInvertedPyramidStairsTerrain (isaac-sim/IsaacLab,
 * {@code terrains/trimesh/mesh_terrains.py}, {@code inverted_pyramid_stairs_terrain}) &mdash; the same box-stacking
 * pattern as {@link PyramidStairsTerrainObject}, mirrored downward instead of upward.
 * <p>
 * See {@link PyramidStairsTerrainObject} for the parameter meanings ({@code sizeX}/{@code sizeY} footprint,
 * {@code stepHeight}/{@code stepWidth} per ring, {@code platformWidth}/{@code borderWidth}, {@code holes}).
 */
public class InvertedPyramidStairsTerrainObject extends CombinedTerrainObject3D
{
   public InvertedPyramidStairsTerrainObject(String name, double sizeX, double sizeY, double stepHeight, double stepWidth, double platformWidth,
                                              double borderWidth, boolean holes, AppearanceDefinition appearance)
   {
      this(name, new RigidBodyTransform(), sizeX, sizeY, stepHeight, stepWidth, platformWidth, borderWidth, holes, appearance);
   }

   public InvertedPyramidStairsTerrainObject(String name, RigidBodyTransformReadOnly transform, double sizeX, double sizeY, double stepHeight,
                                              double stepWidth, double platformWidth, double borderWidth, boolean holes)
   {
      this(name, transform, sizeX, sizeY, stepHeight, stepWidth, platformWidth, borderWidth, holes, YoAppearance.StoneTexture());
   }

   public InvertedPyramidStairsTerrainObject(String name, RigidBodyTransformReadOnly transform, double sizeX, double sizeY, double stepHeight,
                                              double stepWidth, double platformWidth, double borderWidth, boolean holes, AppearanceDefinition appearance)
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

      double totalHeight = (numSteps + 1) * stepHeight;
      double terrainSizeX = sizeX - 2.0 * borderWidth;
      double terrainSizeY = sizeY - 2.0 * borderWidth;

      if (borderWidth > 0.0 && !holes)
         BoxStackingTerrainTools.addFlatBorder(this, transform, sizeX, sizeY, terrainSizeX, terrainSizeY, stepHeight, -0.5 * stepHeight, appearance);

      for (int k = 0; k < numSteps; k++)
      {
         double boxSizeX = holes ? platformWidth : terrainSizeX - 2.0 * k * stepWidth;
         double boxSizeY = holes ? platformWidth : terrainSizeY - 2.0 * k * stepWidth;

         double boxZ = -totalHeight / 2.0 - (k + 1) * stepHeight / 2.0;
         double boxOffset = (k + 0.5) * stepWidth;
         double boxHeight = totalHeight - (k + 1) * stepHeight;

         BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, terrainSizeY / 2.0 - boxOffset, boxZ, boxSizeX, stepWidth, boxHeight, appearance);
         BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, -terrainSizeY / 2.0 + boxOffset, boxZ, boxSizeX, stepWidth, boxHeight, appearance);

         double sideSizeY = holes ? boxSizeY : boxSizeY - 2.0 * stepWidth;
         BoxStackingTerrainTools.addLocalBox(this, transform, terrainSizeX / 2.0 - boxOffset, 0.0, boxZ, stepWidth, sideSizeY, boxHeight, appearance);
         BoxStackingTerrainTools.addLocalBox(this, transform, -terrainSizeX / 2.0 + boxOffset, 0.0, boxZ, stepWidth, sideSizeY, boxHeight, appearance);
      }

      double middleSizeX = terrainSizeX - 2.0 * numSteps * stepWidth;
      double middleSizeY = terrainSizeY - 2.0 * numSteps * stepWidth;
      double middleZ = -totalHeight - stepHeight / 2.0;
      BoxStackingTerrainTools.addLocalBox(this, transform, 0.0, 0.0, middleZ, middleSizeX, middleSizeY, stepHeight, appearance);
   }
}
