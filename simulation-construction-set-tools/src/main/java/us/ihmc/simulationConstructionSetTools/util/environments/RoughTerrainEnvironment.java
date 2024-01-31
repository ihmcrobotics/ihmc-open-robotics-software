package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.ConvexPolytopeTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class RoughTerrainEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround;
   private final Random random = new Random(1989L);

   public RoughTerrainEnvironment()
   {

      flatGround = new CombinedTerrainObject3D("Blop");
      flatGround.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      //flatGround.addTerrainObject(setUpPathRocks3D("Path Rocks", false, 0.0, 0.0));
      flatGround.addTerrainObject(setUpPathRocks3D("Path Rocks1", false, 0.0, 0.0, 5.0, 8.0, 80, 0.3, 30, 0.0, 2.0, 0.5, true, 4));
      flatGround.addTerrainObject(setUpPathRocks3D("Path Rocks2", false, 0.0, 45.0, 4.0, 10.0, 60, 0.3, 50, 0.0, 2.0, 0.5, true, 4));
      flatGround.addTerrainObject(setUpPathRocks3D("Path Flat Rocks", true, 0.0, 90.0));

      flatGround.addTerrainObject(setUpPathStairs("Path Stairs1", 0.0, 0.0));
      flatGround.addTerrainObject(setUpPathStairs("Path Stairs2", 0.0, -45.0, 5.0, 1.5, 0.4, 0.4, 1.2, 11.0));
      flatGround.addTerrainObject(setUpPathStairs("Path Stairs3", 0.0, 45.0, 5.0, 2.5, 0.3, 0.3, 1.0, 9.0));
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
   }

   private TerrainObject3D setUpPathRocks3D(String name, Boolean flatRocks, double heightOffset, double rockYawDegrees)
   {
      double rocksStartY = 4.7;
      double rockPathLength = 8.0;
      int numberOfRocks = 80;
      double maxRockCentroidHeight = 0.3;
      int pointsPerRock = 30;
      // change unevenness of rocks
      double maxAbsXYNormalValue = 0.0;
      double rockFieldWidth = 2.0;
      double rockBoundingBoxWidth = 0.5;
      // fullyRandom Will do a neat grid if set to false;
      boolean fullyRandom = true;
      int rocksPerRow = 4;

      if (flatRocks)
      {
         rocksStartY = 3.5;
         maxRockCentroidHeight = 0.2;
         pointsPerRock = 21;
      }

      return setUpPathRocks3D(name,
                              flatRocks,
                              heightOffset,
                              rockYawDegrees,
                              rocksStartY,
                              rockPathLength,
                              numberOfRocks,
                              maxRockCentroidHeight,
                              pointsPerRock,
                              maxAbsXYNormalValue,
                              rockFieldWidth,
                              rockBoundingBoxWidth,
                              fullyRandom,
                              rocksPerRow);
   }

   private CombinedTerrainObject3D setUpPathStairs(String name, double heightOffset, double courseAngle)
   {
      double startDistance = 4.5;
      double width = 2.0;
      double maxRise = 0.3;
      double minRun = 0.4;
      double maxRun = 1.0;
      double maxLengthOfStairCase = 10.0;

      return setUpPathStairs(name, heightOffset, courseAngle, startDistance, width, maxRise, minRun, maxRun, maxLengthOfStairCase);
   }

   private TerrainObject3D setUpPathRocks3D(String name,
                                            boolean flatRocks,
                                            double heightOffset,
                                            double rockYawDegrees,
                                            double rocksStartY,
                                            double rockPathLength,
                                            int numberOfRocks,
                                            double maxRockCentroidHeight,
                                            int pointsPerRock,
                                            double maxAbsXYNormalValue,
                                            double rockFieldWidth,
                                            double rockBoundingBoxWidth,
                                            boolean fullyRandom,
                                            int rocksPerRow)
   {
      return addRocks3D(name,
                        flatRocks,
                        heightOffset,
                        rockYawDegrees,
                        rocksStartY,
                        rockPathLength,
                        numberOfRocks,
                        maxRockCentroidHeight,
                        pointsPerRock,
                        maxAbsXYNormalValue,
                        rockFieldWidth,
                        rockBoundingBoxWidth,
                        fullyRandom,
                        rocksPerRow);
   }

   private CombinedTerrainObject3D addRocks3D(String name,
                                              boolean flatRocks,
                                              double heightOffset,
                                              double rockYawDegrees,
                                              double rocksStartY,
                                              double rockPathLength,
                                              int numberOfRocks,
                                              double maxRockCentroidHeight,
                                              int pointsPerRock,
                                              double maxAbsXYNormalValue,
                                              double rockFieldWidth,
                                              double rockBoundingBoxWidth,
                                              boolean fullyRandom,
                                              int rocksPerRow)
   {
      CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(name);

      RigidBodyTransform rockRotation = new RigidBodyTransform();
      rockRotation.setRotationYawAndZeroTranslation(Math.toRadians(rockYawDegrees));

      for (int i = 0; i < numberOfRocks; i++)
      {
         double centroidHeight = random.nextDouble() * maxRockCentroidHeight + heightOffset;

         Vector3D normal = generateRandomUpFacingNormal(maxAbsXYNormalValue);

         double[] approximateCentroid = generateRandomApproximateCentroid(i,
                                                                          fullyRandom,
                                                                          rockFieldWidth,
                                                                          rockPathLength,
                                                                          rocksStartY,
                                                                          rocksPerRow,
                                                                          numberOfRocks);

         double[][] vertices = generateRandomRockVertices(approximateCentroid[0],
                                                          approximateCentroid[1],
                                                          pointsPerRock,
                                                          rockBoundingBoxWidth,
                                                          maxRockCentroidHeight,
                                                          numberOfRocks);

         addRock3D(combinedTerrainObject3D, normal, vertices, rockRotation, flatRocks, centroidHeight);
      }

      return combinedTerrainObject3D;
   }

   private Vector3D generateRandomUpFacingNormal(double maxAbsXYNormalValue)
   {
      double normalX = random.nextDouble() * (2.0 * maxAbsXYNormalValue) - maxAbsXYNormalValue;
      double normalY = random.nextDouble() * (2.0 * maxAbsXYNormalValue) - maxAbsXYNormalValue;
      Vector3D normal = new Vector3D(normalX, normalY, 1.0);

      return normal;
   }

   private double[][] generateRandomRockVertices(double approximateCentroidX,
                                                 double approximateCentroidY,
                                                 int pointsPerRock,
                                                 double rockBoundingBoxWidth,
                                                 double maxRockCentroidHeight,
                                                 int numberOfRocks)
   {
      double[][] vertices = new double[pointsPerRock][3];

      for (int j = 0; j < pointsPerRock; j++)
      {
         vertices[j][0] = random.nextDouble() * rockBoundingBoxWidth + approximateCentroidX - rockBoundingBoxWidth / 2.0;
         vertices[j][1] = random.nextDouble() * rockBoundingBoxWidth + approximateCentroidY - rockBoundingBoxWidth / 2.0;
         vertices[j][2] = random.nextDouble() * maxRockCentroidHeight - maxRockCentroidHeight / 2.0;
      }

      return vertices;
   }

   public static void addRock3D(CombinedTerrainObject3D combinedTerrainObject,
                                Vector3D normal,
                                double[][] vertices,
                                RigidBodyTransform rockRotation,
                                boolean flatRocks,
                                double centroidHeight)
   {
      YoAppearanceMaterial rockAppearance = new YoAppearanceMaterial();
      rockAppearance.setSpecularColor(0.5f, 0.5f, 0.5f);
      rockAppearance.setDiffuseColor(0.4f, 0.4f, 0.4f);
      rockAppearance.setShininess(6.0f);
      rockAppearance.setAmbientColor(0.16f, 0.18f, 0.2f);

      if (flatRocks)
      {
         ArrayList<Point2D> vertexPoints = new ArrayList<Point2D>();

         for (double[] point : vertices)
         {
            Point2D point2d = new Point2D(point[0], point[1]);
            vertexPoints.add(point2d);
         }

         ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertexPoints));
         convexPolygon.applyTransform(rockRotation);
         RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, rockAppearance);
         combinedTerrainObject.addTerrainObject(rock);
      }
      else
      {
         ArrayList<Point3D> vertexPoints = new ArrayList<Point3D>();

         for (double[] point : vertices)
         {
            Point3D point3d = new Point3D(point[0], point[1], point[2]);
            vertexPoints.add(point3d);
         }

         ConvexPolytope3D convexPolytope = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertexPoints));

         convexPolytope.applyTransform(rockRotation);
         ConvexPolytopeTerrainObject rock = new ConvexPolytopeTerrainObject(convexPolytope, rockAppearance);
         combinedTerrainObject.addTerrainObject(rock);
      }

   }

   private double[] generateRandomApproximateCentroid(int position,
                                                      boolean fullyRandom,
                                                      double rockFieldWidth,
                                                      double rockPathLength,
                                                      double rocksStartY,
                                                      int rocksPerRow,
                                                      int numberOfRocks)
   {
      double[] approximateCentroid = new double[2];

      if (fullyRandom)
      {
         approximateCentroid[0] = random.nextDouble() * rockFieldWidth - rockFieldWidth / 2.0;
         approximateCentroid[1] = random.nextDouble() * rockPathLength + rocksStartY;
      }
      else
      {
         int row = position / rocksPerRow;
         int rows = numberOfRocks / rocksPerRow;
         double distancePerRow = rockPathLength / ((double) rows - 1);
         approximateCentroid[1] = rocksStartY + distancePerRow * row;

         int positionOnRow = position - row * rocksPerRow;
         approximateCentroid[0] = rockFieldWidth * positionOnRow / rocksPerRow - rockFieldWidth / 2.0;
      }

      return approximateCentroid;
   }

   private CombinedTerrainObject3D setUpPathStairs(String name,
                                                   double heightOffset,
                                                   double courseAngle,
                                                   double startDistance,
                                                   double width,
                                                   double maxRise,
                                                   double minRun,
                                                   double maxRun,
                                                   double maxLengthOfStairCase)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      AppearanceDefinition color = YoAppearance.DarkGray();
      double totalLength = startDistance;
      double lastRise = 0.0;

      while (totalLength <= maxLengthOfStairCase)
      {
         double run = EuclidCoreRandomTools.nextDouble(random, minRun, maxRun);
         double rise = EuclidCoreRandomTools.nextDouble(random, maxRise);

         rise = lastRise + rise < 0.0 ? lastRise - rise : lastRise + rise;
         lastRise = rise;

         double[] newPoint = rotateAroundOrigin(new double[] {totalLength + run / 2.0, 0}, courseAngle);
         setUpWall(combinedTerrainObject, newPoint, width, run, rise, courseAngle, heightOffset, color);

         totalLength += run;
      }

      return combinedTerrainObject;
   }

   private static double[] rotateAroundOrigin(double[] xy, double angdeg)
   {
      double x = xy[0];
      double y = xy[1];
      double[] newPoint = new double[2];
      double angRad = Math.toRadians(angdeg);
      newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
      newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);
      return newPoint;
   }

   private static void setUpWall(CombinedTerrainObject3D combinedTerrainObject,
                                 double[] xy,
                                 double width,
                                 double length,
                                 double height,
                                 double yawDegrees,
                                 double heightOffset,
                                 AppearanceDefinition app)
   {
      double x = xy[0];
      double y = xy[1];
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      location.getTranslation().set(new Vector3D(x, y, height / 2 + heightOffset));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }
}