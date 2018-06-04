package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.ArrayList;

public class ObstacleCoursePlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   private final ArrayList<PlanarRegionsList> planarRegionsLists = new ArrayList<>();
   private final ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

   public ObstacleCoursePlanarRegionEnvironment()
   {
      // ground plane
      generator.translate(3.25, 0.0, -0.01);
      generator.addRectangle(9.5, 16.0);
      generator.translate(-3.25, 0.0, 0.0);

      generator.translate(-4.0, -4.6, 0.0);
      generator.addRectangle(8.0, 6.8);
      generator.translate(4.0, 4.6, 0.0);

      generator.translate(-4.0, 4.6, 0.0);
      generator.addRectangle(8.0, 6.8);
      generator.translate(4.0, -4.6, 0.0);

      generator.translate(-6.7, 0.0, 0.0);
      generator.addRectangle(2.6, 2.4);
      generator.translate(6.7, 0.0, 0.0);
      addPlanarRegionsToTerrain(YoAppearance.RGBColor(110 / 256.0, 121 / 256.0, 121 / 256.0));

      // staircase and ramps
      generator.identity();
      generator.translate(1.5, 0.0, 0.0);
      double stepHeight = 0.1;
      double stepLength = 0.30;
      int numberOfSteps = 6;
      double startingBlockLength = 1.0;

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (i + 1) * stepHeight);
      }

      generator.translate(0.5 * (startingBlockLength + stepLength) - 0.5 * stepLength, 0.5 * startingBlockLength, 0.0);
      generator.addCubeReferencedAtBottomMiddle(startingBlockLength + stepLength, 2.0 * startingBlockLength, numberOfSteps * stepHeight);
      generator.translate(0.5 * (startingBlockLength - stepLength), - 0.5 * startingBlockLength, 0.0);

      for (int i = 1; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (numberOfSteps - i) * stepHeight);
      }

      // ramps
      generator.identity();
      generator.translate(1.5, 1.0, 0.0);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1.4, stepHeight * numberOfSteps);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, stepHeight * numberOfSteps);
      generator.addRectangle(startingBlockLength + stepLength, startingBlockLength);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, -stepHeight * numberOfSteps);
      generator.rotate(Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1., stepHeight * numberOfSteps);

      // cinder blocks
      generator.identity();
      generator.translate(-1.5, 0.0, -0.05);
      generator.rotate(Math.PI, Axis.Z);
      PlanarRegionsListExamples.generateCinderBlockField(generator, 0.4, 0.1, 9, 6, 0.05);

      // stepping stones
      generator.identity();
      double steppingStoneHeight = 0.3;
      generator.translate(0.0, 1.5, 0.0);
      generator.rotate(0.5 * Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle(1.0, 1.0, steppingStoneHeight);
      generator.translate(1.4, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.8, 1.0, steppingStoneHeight);

      generator.translate(-0.3, 0.1, 0.0); // shift the stepping stones a little to align with starting block

      generator.translate(0.925, 0.125, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.25, 0.25, steppingStoneHeight);
      generator.translate(-0.925, -0.125, 0.0);

      generator.translate(1.1, -0.375, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.6, 0.25, steppingStoneHeight);
      generator.translate(-1.1, 0.375, 0.0);

      generator.translate(1.25, 0.05, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.35, 0.2, steppingStoneHeight);
      generator.translate(-1.25, -0.05, 0.0);

      generator.translate(1.65, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.3, steppingStoneHeight);
      generator.translate(-1.65, -0.2, 0.0);

      generator.translate(1.65, -0.275, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, steppingStoneHeight);
      generator.translate(-1.65, 0.275, 0.0);

      generator.translate(1.975, 0.075, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.25, 0.35, steppingStoneHeight);
      generator.translate(-1.975, -0.075, 0.0);

      generator.translate(2.0, -0.35, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.35, 0.35, steppingStoneHeight);
      generator.translate(-2.0, 0.35, 0.0);

      generator.translate(1.5 + 0.5 * 0.3 + 0.1 + 0.6, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(1.2, 1.0, steppingStoneHeight);
      generator.translate(0.0, -(0.5 + 1.0), 0.0);
      generator.rotate(0.5 * Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle(1.0, 1.2, steppingStoneHeight);

      addPlanarRegionsToTerrain(YoAppearance.Gray());
   }

   private void addPlanarRegionsToTerrain(AppearanceDefinition appearance)
   {
      planarRegionsLists.add(generator.getPlanarRegionsList().copy());
      appearances.add(appearance);
      generator.reset();
   }

   @Override
   public void generateEnvironment()
   {
      PlanarRegionsList[] planarRegionsLists = new PlanarRegionsList[this.planarRegionsLists.size()];
      AppearanceDefinition[] appearances = new AppearanceDefinition[this.appearances.size()];

      for (int i = 0; i < planarRegionsLists.length; i++)
      {
         planarRegionsLists[i] = this.planarRegionsLists.get(i);
         appearances[i] = this.appearances.get(i);
      }

      environment = new PlanarRegionsListDefinedEnvironment("obstacleCourse", planarRegionsLists, appearances, 1e-2, false);
      hasBeenGenerated = true;
   }

   @Override
   public PlanarRegionsList getPlanarRegionsList()
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      for (int i = 0; i < planarRegionsLists.size(); i++)
      {
         planarRegionsList.getPlanarRegionsAsList().addAll(planarRegionsLists.get(i).getPlanarRegionsAsList());
      }

      return planarRegionsList;
   }

   //   @Override
//   public TerrainObject3D getTerrainObject3D()
//   {
//      TerrainObject3D planarRegionsTerrain = super.getTerrainObject3D();
//      CombinedTerrainObject3D combinedTerrain = new CombinedTerrainObject3D("obstacleCourse");
//      combinedTerrain.addTerrainObject(planarRegionsTerrain);
//      combinedTerrain.addTerrainObject(new BumpyGroundTerrainObject());
//      return combinedTerrain;
//   }
//
//   // adding bumpy ground as a terrain object slows the sim down too much
//   private class BumpyGroundTerrainObject extends BumpyGroundProfile implements TerrainObject3D
//   {
//      private final Graphics3DObject graphics = new Graphics3DObject();
//      private static final double xAmp1 = 0.02, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
//      private static final double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.02, yFreq2 = 0.37;
//
//      private BumpyGroundTerrainObject()
//      {
//         super(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, -3.0, 3.0, -6.0, -3.0);
//         graphics.addHeightMap(this, 100, 100, YoAppearance.DarkGreen());
//      }
//
//      @Override
//      public Graphics3DObject getLinkGraphics()
//      {
//         return graphics;
//      }
//   }
}
