package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCDemo01Environment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final Random random = new Random(1989L);
   
   private static final double WALL_START_X = 1.0;
   private static final double WALL_LENGTH = 5.0;
   private static final double WALL_Y = -0.7;
   private static final double WALL_THICKNESS = 0.05;
   private static final double PILLAR_WIDTH = 0.3;
   private static final int NUM_PILLARS = 6;
   
   private static final int NUM_ROCKS = 75;
   private static final double MAX_ROCK_CENTROID_HEIGHT = 0.35;
   private static final int POINTS_PER_ROCK = 21;
   private static final double MAX_ABS_XY_NORMAL_VALUE = 0.2;
   private static final double ROCK_FIELD_WIDTH = 1.0;
   private static final double ROCK_BOUNDING_BOX_WIDTH = 0.3;
   
   private static final boolean FULLY_RANDOM = true; // Will do a neat grid if set to false;
   private static final int ROCKS_PER_ROW = 5;
   
   public DRCDemo01Environment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D("Rocks with a wall");
      addWall();
      addPillars();
      addRocks();
      addGround();
   }


   private void addGround()
   {
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0);
   }

   private void addRocks()
   {
      for(int i = 0; i < NUM_ROCKS; i++)
      {
         double centroidHeight = random.nextDouble() * MAX_ROCK_CENTROID_HEIGHT;
         Vector3d normal = generateRandomUpFacingNormal();

         double[] approximateCentroid = generateRandomApproximateCentroid(i);

         double[][] vertices = generateRandomRockVertices(approximateCentroid[0], approximateCentroid[1]);

         addRock(normal, centroidHeight, vertices);         
      }
   }

   private double[] generateRandomApproximateCentroid(int position)
   {
      double[] approximateCentroid = new double[2];
      
      if(FULLY_RANDOM)
      {
         approximateCentroid[0] = random.nextDouble() * WALL_LENGTH + WALL_START_X;
         approximateCentroid[1] = random.nextDouble() * ROCK_FIELD_WIDTH - ROCK_FIELD_WIDTH/2.0;
      }
      else
      {
         int row = position / ROCKS_PER_ROW;
         int rows = NUM_ROCKS / ROCKS_PER_ROW;
         double distancePerRow = WALL_LENGTH / ((double) rows - 1);
         approximateCentroid[0] = WALL_START_X + distancePerRow * row;

         int positionOnRow = position - row * ROCKS_PER_ROW;
         approximateCentroid[1] =  ROCK_FIELD_WIDTH * positionOnRow/ROCKS_PER_ROW- ROCK_FIELD_WIDTH / 2.0;
      }
      return approximateCentroid;
   }

   private Vector3d generateRandomUpFacingNormal()
   {
      double normalX = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      double normalY = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      Vector3d normal = new Vector3d(normalX, normalY, 1.0);
      return normal;
   }

   private double[][] generateRandomRockVertices(double approximateCentroidX, double approximateCentroidY)
   {
      double[][] vertices = new double[POINTS_PER_ROCK][2];

      for(int j = 0; j < POINTS_PER_ROCK; j++)
      {
         vertices[j][0] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidX - ROCK_BOUNDING_BOX_WIDTH / 2.0;
         vertices[j][1] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidY - ROCK_BOUNDING_BOX_WIDTH / 2.0;
      }
      return vertices;
   }

   private void addRock(Vector3d normal, double centroidHeight, double[][] vertices)
   {
      ArrayList<Point2d> vertexPoints = new ArrayList<Point2d>();
      
      for (double[] point : vertices)
      {
         Point2d point2d = new Point2d(point);
         vertexPoints.add(point2d);
      }
      
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
      RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, YoAppearance.Red());
      this.combinedTerrainObject.addTerrainObject(rock);
   }

   private void addWall()
   {
      Vector3d normal = new Vector3d(0.0, 0.0, 1.0);
      double centroidHeight = 2.0;
      ArrayList<Point2d> pointList = new ArrayList<Point2d>();

      Point2d wallPoint0 = new Point2d(WALL_START_X, WALL_Y);
      Point2d wallPoint1 = new Point2d(WALL_START_X + WALL_LENGTH, WALL_Y);
      Point2d wallPoint2 = new Point2d(WALL_START_X + WALL_LENGTH, WALL_Y + Math.signum(WALL_Y) * WALL_THICKNESS);
      Point2d wallPoint3 = new Point2d(WALL_START_X, WALL_Y + Math.signum(WALL_Y) * WALL_THICKNESS);
      pointList.add(wallPoint0);
      pointList.add(wallPoint1);
      pointList.add(wallPoint2);
      pointList.add(wallPoint3);
      
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(pointList);
      RotatableConvexPolygonTerrainObject rightWall = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, YoAppearance.Brown());
      combinedTerrainObject.addTerrainObject(rightWall);
   }
   
   private void addPillars()
   {
      Vector3d normal = new Vector3d(0.0, 0.0, 1.0);
      double centroidHeight = 2.0;
      
      Point2d bottomLeft = new Point2d(-PILLAR_WIDTH/2.0, PILLAR_WIDTH/2.0);
      Point2d bottomRight = new Point2d(-PILLAR_WIDTH/2.0, -PILLAR_WIDTH/2.0);
      Point2d topLeft = new Point2d(PILLAR_WIDTH/2.0, PILLAR_WIDTH/2.0);
      Point2d topRight = new Point2d(PILLAR_WIDTH/2.0, -PILLAR_WIDTH/2.0);
      
      double pillarDistance = WALL_LENGTH/(NUM_PILLARS - 1.0);
      Vector2d offset = new Vector2d(0.0, -WALL_Y + PILLAR_WIDTH/2.0);
      
      for(int i = 0; i < NUM_PILLARS; i++)
      {
         ArrayList<Point2d> points = new ArrayList<Point2d>();
         offset.setX(WALL_START_X + pillarDistance * i);

         Point2d localBottomLeft = new Point2d();
         localBottomLeft.add(bottomLeft, offset);
         Point2d localBottomRight = new Point2d();
         localBottomRight.add(bottomRight, offset);
         Point2d localTopLeft = new Point2d();
         localTopLeft.add(topLeft, offset);
         Point2d localTopRight = new Point2d();
         localTopRight.add(topRight, offset);

         points.add(localBottomLeft);
         points.add(localBottomRight);
         points.add(localTopLeft);
         points.add(localTopRight);
         
         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(points);
         AppearanceDefinition appearance = YoAppearance.Brown();
//         YoAppearance.makeTransparent(appearance, 0.7f);
         RotatableConvexPolygonTerrainObject pillar = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, appearance);
         combinedTerrainObject.addTerrainObject(pillar);
      }
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public ArrayList<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

}
