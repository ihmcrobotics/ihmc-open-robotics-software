package us.ihmc.avatar.environments;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.io.File;
import java.nio.file.Paths;
import java.util.Random;

public class BehaviorPlanarRegionEnvironments extends PlannerTestEnvironments
{
   public static final double CINDER_SLOPE_ANGLE = 13.0;
   public static final double Z_STEP_UP_PER_ROW = 0.10;
   private static double cinderSquareSurfaceSize = 0.395;
   private static double cinderThickness = 0.145;
   public static double topRegionHeight = 5 * Z_STEP_UP_PER_ROW - cinderThickness;
   public static double topPlatformHeight = topRegionHeight + cinderThickness + 0.07;
   private static double superGridSize = cinderSquareSurfaceSize * 3;
   private static double groundSize = 20.0;
   private static int greenId = 6;

   public static PlanarRegionsList flatGround()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(groundSize, groundSize);
      return generator.getPlanarRegionsList();
   }

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
                                                         Math.toRadians(CINDER_SLOPE_ANGLE),
                                                         Math.toRadians(CINDER_SLOPE_ANGLE),
                                                         0.05,
                                                         false);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createStairs()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(0.75, 0.75);
      generator.translate(0.75, 0.0, -0.25);
      generator.addRectangle(0.75, 0.75);
      generator.translate(0.75, 0.0, -0.25);
      generator.addRectangle(0.75, 0.75);
      generator.translate(0.75, 0.0, -0.25);
      generator.addRectangle(0.75, 0.75);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createUpDownOpenHouseRegions()
   {
      Random random = new Random(8349829898174L);
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(groundSize, groundSize); // ground TODO form around terrain with no overlap?

      generator.translate(1.0, -superGridSize / 2, 0.0);

      addTopFlatRegionOld(generator);
      addPlusFormationSlopes(random, generator);
      addXFormationSlopes(random, generator);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createRoughUpAndDownStepsWithFlatTop()
   {
      return createRoughUpAndDownStepsWithFlatTop(true);
   }

   public static PlanarRegionsList createRoughUpAndDownStepsWithFlatTop(boolean createGroundPlane)
   {
      return generate((random, generator) -> {
         offsetGrid(generator, 1.0, -0.5, 0.0, () ->
         {
            offsetGrid(generator, 0.0, 0.0, 0.0, () ->
            {
               generateAngledCinderBlockSteps(random, generator);
            });
            offsetGrid(generator, 1.0, 0.0, topRegionHeight, () ->
            {
               addTopFlatRegion(generator);
            });
            offsetGrid(generator, 3.0, 1.0, 0.0, () ->
            {
               rotate(generator, Math.PI, () ->
               {
                  generateAngledCinderBlockSteps(random, generator);
               });
            });
         });
      }, createGroundPlane);
   }

   public static PlanarRegionsList createFlatUpAndDownStepsWithFlatTop()
   {
      return generate((random, generator) -> {
         offsetGrid(generator, 1.0, -0.5, 0.0, () ->
         {
            offsetGrid(generator, 0.0, 0.0, 0.0, () ->
            {
               generateFlatCinderBlockSteps(generator);
            });
            offsetGrid(generator, 1.0, 0.0, topRegionHeight, () ->
            {
               addTopFlatRegion(generator);
            });
            offsetGrid(generator, 3.0, 1.0, 0.0, () ->
            {
               rotate(generator, Math.PI, () ->
               {
                  generateFlatCinderBlockSteps(generator);
               });
            });
         });
      }, true);
   }

   public static PlanarRegionsList generateStartingBlockRegions(RigidBodyTransform startingPose)
   {
      double platformXY = 1.0;
      double platformHeight = 0.3;

      double blockXY = 0.35;
      double blockHeight = 0.15;
      double blockSpacing = 0.1;

      double maxBlockAngle = Math.toRadians(20.0);
      double lastRowMaxBlockAngle = Math.toRadians(10.0);

      Random random = new Random(328903);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setTransform(startingPose);
      generator.setId(250);
      generator.addCubeReferencedAtBottomMiddle(platformXY, platformXY, platformHeight);
      generator.translate(0.5 * (platformXY + blockXY), 3.0 * (blockXY + blockSpacing), 0.0);

      for (int i = 0; i < 5; i++)
      {
         double height = platformHeight - blockHeight - 0.05 * (i + 1);
         generator.translate(0.0, -4.0 * (blockXY + blockSpacing), 0.0);

         if (i % 2 == 0)
         {
            generator.translate(0.0, -0.5 * (blockXY + blockSpacing), 0.0);
         }
         else
         {
            generator.translate(0.0, 0.5 * (blockXY + blockSpacing), 0.0);
         }

         for (int j = 0; j < 4; j++)
         {
            double rotationAxisDirection = 2.0 * Math.PI * random.nextDouble();
            double rotationAngle = (i == 4 ? lastRowMaxBlockAngle : maxBlockAngle) * random.nextDouble();
            double yawRotation = 2.0 * Math.PI * random.nextDouble();

            Vector3D rotationAxis = new Vector3D(Math.cos(rotationAxisDirection), Math.sin(rotationAxisDirection), 0.0);
            rotationAxis.scale(rotationAngle);
            AxisAngle axisAngle = new AxisAngle(rotationAxis, rotationAngle);
            RotationMatrix rotationMatrix = new RotationMatrix();
            axisAngle.get(rotationMatrix);

            generator.translate(0.0, 0.0, height);
            generator.rotate(rotationMatrix);
            generator.rotate(yawRotation, Axis3D.Z);
            generator.addCubeReferencedAtBottomMiddle(blockXY, blockXY, blockHeight);
            generator.rotate(-yawRotation, Axis3D.Z);
            rotationMatrix.invert();
            generator.rotate(rotationMatrix);
            generator.translate(0.0, 0.0, -height);

            generator.translate(0.0, blockXY + blockSpacing, 0.0);
         }

         generator.translate(blockXY, 0.0, 0.0);
      }

      return generator.getPlanarRegionsList();
   }

   private static PlanarRegionsList generate(GenerationInterface generationInterface, boolean createGroundPlane)
   {
      Random random = new Random(8349829898174L);
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      if (createGroundPlane)
      {
         generator.setId(greenId);
         generator.addRectangle(groundSize, groundSize); // ground TODO form around terrain with no overlap?
      }

      generationInterface.generate(random, generator);
      return generator.getPlanarRegionsList();
   }

   interface GenerationInterface
   {
      void generate(Random random, PlanarRegionsListGenerator generator);
   }

   public static PlanarRegionsList createUpDownTwoHighWithFlatBetween()
   {
      Random random = new Random(8349829898174L);
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(groundSize, groundSize); // ground TODO form around terrain with no overlap?

      generator.translate(1.0, -superGridSize / 2, 0.0);
      addTopFlatRegionOld(generator);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);

      generator.rotate(Math.PI, Axis3D.Z);
      generator.translate(2.0, -1.2, 0.0);
      addTopFlatRegionOld(generator);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createUpDownFourHighWithFlatCenter()
   {
      Random random = new Random(8349829898174L);
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setId(greenId);
      generator.addRectangle(groundSize, groundSize); // ground TODO form around terrain with no overlap?

      generator.rotate(Math.PI/4, Axis3D.Z);

      double xSize = 0.5;
      double ySize = xSize + 2.0;
      generator.translate(xSize * superGridSize, -ySize * superGridSize, 0.0);
      addHighCorner(random, generator);
      generator.translate(-xSize * superGridSize, ySize * superGridSize, 0.0);

      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(xSize * superGridSize, -ySize * superGridSize, 0.0);
      addHighCorner(random, generator);
      generator.translate(-xSize * superGridSize, ySize * superGridSize, 0.0);

      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(xSize * superGridSize, -ySize * superGridSize, 0.0);
      addHighCorner(random, generator);
      generator.translate(-xSize * superGridSize, ySize * superGridSize, 0.0);

      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(xSize * superGridSize, -ySize * superGridSize, 0.0);
      addHighCorner(random, generator);
      generator.translate(-xSize * superGridSize, ySize * superGridSize, 0.0);

      return generator.getPlanarRegionsList();
   }

   private static void addHighCorner(Random random, PlanarRegionsListGenerator generator)
   {
      addTopFlatRegionOld(generator);

      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);

      generator.translate(0.0, -superGridSize, 0.0);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
   }

   private static void addXFormationSlopes(Random random, PlanarRegionsListGenerator generator)
   {
      generator.translate(0.0, -superGridSize, 0.0);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(3*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateCorner(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, 0.0, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
   }

   private static void addPlusFormationSlopes(Random random, PlanarRegionsListGenerator generator)
   {
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
      generateAngledCinderBlockSteps(random, generator, cinderSquareSurfaceSize, cinderThickness);
      generator.translate(2*superGridSize, -superGridSize, 0.0);
      generator.rotate(Math.PI/2, Axis3D.Z);
   }

   private static void addTopFlatRegionOld(PlanarRegionsListGenerator generator)
   {
      offsetGrid(generator, superGridSize, 0.0, topRegionHeight, () -> {
         generator.addCubeReferencedAtBottomNegativeXYCorner(superGridSize, superGridSize, cinderThickness);
      });
   }

   private static void addTopFlatRegion(PlanarRegionsListGenerator generator)
   {
      generator.addCubeReferencedAtBottomNegativeXYCorner(superGridSize, superGridSize, cinderThickness);
   }

   private static void generateAngledCinderBlockSteps(Random random, PlanarRegionsListGenerator generator)
   {
      PlanarRegionsListExamples.generateCinderBlockSlope(generator,
                                                         random,
                                                         cinderSquareSurfaceSize,
                                                         cinderThickness,
                                                         3,
                                                         3,
                                                         Z_STEP_UP_PER_ROW,
                                                         0.0,
                                                         0.0,
                                                         Math.toRadians(CINDER_SLOPE_ANGLE),
                                                         0.0);
   }
   
   private static void generateFlatCinderBlockSteps(PlanarRegionsListGenerator generator)
   {
      PlanarRegionsListExamples.generateFlatCinderBlockSteps(generator, cinderSquareSurfaceSize, cinderThickness, 3, 3, 0.11 + 0.04);
   }
   private static void generateAngledCinderBlockSteps(Random random, PlanarRegionsListGenerator generator, double cinderSquareSurfaceSize, double cinderThickness)
   {
      PlanarRegionsListExamples.generateCinderBlockSlope(generator,
                                                         random,
                                                         cinderSquareSurfaceSize,
                                                         cinderThickness,
                                                         3,
                                                         3,
                                                         Z_STEP_UP_PER_ROW,
                                                         0.0,
                                                         0.0,
                                                         Math.toRadians(CINDER_SLOPE_ANGLE),
                                                         0.0);
   }

   private static void generateCorner(Random random, PlanarRegionsListGenerator generator, double cinderSquareSurfaceSize, double cinderThickness)
   {
      PlanarRegionsListExamples.generateCinderBlockCornerSlope(generator,
                                                               random,
                                                               cinderSquareSurfaceSize,
                                                               cinderThickness,
                                                               3,
                                                               3,
                                                               Z_STEP_UP_PER_ROW,
                                                               0.0,
                                                               0.0,
                                                               Math.toRadians(0.0),
                                                               0.0);
   }

   private static void offsetGrid(PlanarRegionsListGenerator generator, double x, double y, Runnable runnable)
   {
      offsetGrid(generator, x, y, 0.0, runnable);
   }

   private static void offsetGrid(PlanarRegionsListGenerator generator, double x, double y, double z, Runnable runnable)
   {
      generator.translate(x * superGridSize, y * superGridSize, z * superGridSize);
      runnable.run();
      generator.translate(-x * superGridSize, -y * superGridSize, -z * superGridSize);
   }

   private static void rotate(PlanarRegionsListGenerator generator, double yaw, Runnable runnable)
   {
      generator.rotate(yaw, Axis3D.Z);
      runnable.run();
      generator.rotate(-yaw, Axis3D.Z);
   }

   public static void main(String[] args)
   {
      PlanarRegionsList roughUpAndDownStairsWithFlatTop = createRoughUpAndDownStepsWithFlatTop();
      PlanarRegionFileTools.exportPlanarRegionData(Paths.get(System.getProperty("user.home") + File.separator + "PlanarRegions" + File.separator), roughUpAndDownStairsWithFlatTop);
   }

   public static PlanarRegionsList realDataFromAtlasSLAMDataset20190710()
   {
      PlanarRegionsList map = PlanarRegionsList.flatGround(10.0);
      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      map = PlanarRegionSLAM.slam(map, loadDataSet("20190710_174025_PlanarRegion"), parameters).getMergedMap();
      map = PlanarRegionSLAM.slam(map, loadDataSet("IntentionallyDrifted"), parameters).getMergedMap();
      map = PlanarRegionSLAM.slam(map, loadDataSet("20190710_174422_PlanarRegion"), parameters).getMergedMap();
      return map;
   }

   private static PlanarRegionsList loadDataSet(String dataSetName)
   {
      return PlanarRegionFileTools.importPlanarRegionData(ClassLoader.getSystemClassLoader(),
                                                          DataSetIOTools.DATA_SET_DIRECTORY_PATH + "/20190710_SLAM_PlanarRegionFittingExamples/" + dataSetName);
   }
}
