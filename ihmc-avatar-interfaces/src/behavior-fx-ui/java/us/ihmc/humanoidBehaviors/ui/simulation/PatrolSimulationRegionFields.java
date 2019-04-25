package us.ihmc.humanoidBehaviors.ui.simulation;

import us.ihmc.euclid.Axis;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.Random;

public class PatrolSimulationRegionFields
{
   private static double cinderSquareSurfaceSize = 0.395;
   private static double cinderThickness = 0.145;
   private static double topRegionHeight = 0.56 - cinderThickness;
   private static double superGridSize = cinderSquareSurfaceSize * 3;
   private static double topSquareSize = 20.0;
   private static int greenId = 6;

   public static PlanarRegionsList createTraversalRegionsRegions()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      PlanarRegionsListExamples.generateCinderBlockField(generator,
                                                         0.4,
                                                         0.1,
                                                         5,
                                                         6,
                                                         0.02,
                                                         -0.03,
                                                         1.5,
                                                         0.0,
                                                         Math.toRadians(15.0),
                                                         Math.toRadians(15.0),
                                                         0.05,
                                                         false);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createUpDownOpenHouseRegions()
   {
      Random random = new Random(System.nanoTime());
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(topSquareSize, topSquareSize); // ground TODO form around terrain with no overlap?
      addTopFlatRegion(generator);
      addPlusFormationSlopes(random, generator);
      addXFormationSlopes(random, generator);
      return generator.getPlanarRegionsList();
   }

   private static void addXFormationSlopes(Random random, PlanarRegionsListGenerator generator)
   {
      generator.translate(0.0, -superGridSize, 0.0);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
   }

   private static void addPlusFormationSlopes(Random random, PlanarRegionsListGenerator generator)
   {
      generateSlope(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateSlope(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateSlope(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
      generateSlope(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis.Z);
   }

   private static void addTopFlatRegion(PlanarRegionsListGenerator generator)
   {
      generator.translate(superGridSize, 0.0, topRegionHeight);
      generator.addCubeReferencedAtBottomNegativeXYCorner(superGridSize, superGridSize, cinderThickness);
      generator.translate(-superGridSize, 0.0, -topRegionHeight);
   }

   private static void generateSlope(Random random, PlanarRegionsListGenerator generator, double cinderSquareSurfaceSize, double cinderThickness)
   {
      PlanarRegionsListExamples.generateCinderBlockSlope(generator,
                                                         random, cinderSquareSurfaceSize, cinderThickness,
                                                         3,
                                                         3,
                                                         0.15,
                                                         0.0,
                                                         0.0,
                                                         Math.toRadians(15.0),
                                                         0.0);
   }

   private static void generateCorner(Random random, PlanarRegionsListGenerator generator, double cinderSquareSurfaceSize, double cinderThickness)
   {
      PlanarRegionsListExamples.generateCinderBlockCornerSlope(generator,
                                                               random, cinderSquareSurfaceSize, cinderThickness,
                                                               3,
                                                               3,
                                                               0.15,
                                                               0.0,
                                                               0.0,
                                                               Math.toRadians(15.0),
                                                               0.0);
   }
}
