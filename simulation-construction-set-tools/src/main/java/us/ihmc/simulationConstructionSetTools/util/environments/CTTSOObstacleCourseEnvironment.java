package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableRampTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.SimpleTableTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CTTSOObstacleCourseEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D;

   private static final CourseType type = CourseType.B;

   enum CourseType
   {
      A, B, C;
   }

   private static final double flatGridHeight = 0.3;
   private static final double gridLength = 1.25;
   private static final double gridWidth = 1.0;

   private static final int gridDimensionX = 3;
   private static final int gridDimensionY = 7;

   private static final double worldLength = gridDimensionX * gridLength;
   private static final double worldWidth = gridDimensionY * gridWidth;

   private static final double tableHeight = 0.5;
   private static final double tableThickness = 0.05;
   private static final double tableLength = 0.7;
   private static final double tableWidth = 0.5;

   private static final double curbHeight = 0.2;
   private static final double fillet = 0.1;

   private static final double bumpHeight = 0.2;
   private static final double bumplength = 0.5;
   private static final double bumpWidth = 1.5;

   private static final double bollardHeight = 0.7;
   private static final double bollardRadius = 0.075;

   private static final double potholeDepth = 0.3;
   private static final double potholeRadius = 0.4;

   public CTTSOObstacleCourseEnvironment()
   {
      combinedTerrainObject3D = new CombinedTerrainObject3D("CTTSOObstacleCourseEnvironment");

      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      switch (type)
      {
      case A:
         break;
      case B:
         combinedTerrainObject3D.addTerrainObject(setUpInclinedSurface(0, 0, 0, 3));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 0));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 0));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 5));
         combinedTerrainObject3D.addTerrainObject(setUpBollard(0, 4, new Point2D(0.4, 0.0), bollardHeight, bollardRadius));
         combinedTerrainObject3D.addTerrainObject(setUpBollard(0, 4, new Point2D(-0.4, 0.0), bollardHeight, bollardRadius));

         combinedTerrainObject3D.addTerrainObject(setUpStormGrate(1, 5));

         //         combinedTerrainObject3D.addTerrainObject(setUpGridWithPotholes(1, 1));
         //         combinedTerrainObject3D.addTerrainObject(setUpGridWithPotholes(2, 1));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 2));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 2));

         combinedTerrainObject3D.addTerrainObject(setUpCurb(1, 3));
         combinedTerrainObject3D.addTerrainObject(setUpCurb(2, 3));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 6));
         combinedTerrainObject3D.addTerrainObject(setUpTable(2, 6, new Point2D(-0.0, 0.0)));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 5));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 6));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 6));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 6));
         break;
      case C:
         break;
      }
   }

   private TerrainObject3D setUpFlatGrid(int row, int column)
   {
      AppearanceDefinition gridAppearance = YoAppearance.Grey();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(0.0, 0.0, -flatGridHeight / 2);

      return new RotatableBoxTerrainObject(new Box3D(location, gridLength, gridWidth, flatGridHeight), gridAppearance);
   }

   private TerrainObject3D setUpBollard(int row, int column, Point2D positionInGrid, double height, double radius)
   {
      AppearanceDefinition bollardAppearance = YoAppearance.Yellow();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), height / 2);

      return new CylinderTerrainObject(location, height, radius, bollardAppearance);
   }

   private TerrainObject3D setUpTable(int row, int column, Point2D positionInGrid)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), 0.0);

      double xStart = location.getTranslationX() - tableLength / 2;
      double yStart = location.getTranslationY() - tableWidth / 2;
      double xEnd = location.getTranslationX() + tableLength / 2;
      double yEnd = location.getTranslationY() + tableWidth / 2;
      double zStart = tableHeight - tableThickness + getWorldCoordinate(row, column).getZ();
      double zEnd = tableHeight + getWorldCoordinate(row, column).getZ();

      return new SimpleTableTerrainObject(xStart, yStart, xEnd, yEnd, zStart, zEnd);
   }

   private CombinedTerrainObject3D setUpCurb(int row, int column)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("CurbGrid");

      combinedTerrainObject.addTerrainObject(setUpFlatGrid(row, column));

      AppearanceDefinition curbAppearance = YoAppearance.DarkSlateGrey();
      AppearanceDefinition edgeAppearance = YoAppearance.DarkGrey();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(0.0, 0.0, curbHeight / 2);

      combinedTerrainObject.addRotatableBox(location, gridLength, gridWidth - curbHeight * 2, curbHeight, curbAppearance);

      for (RobotSide robotSide : RobotSide.values)
      {
         double filletRadius = curbHeight * fillet;
         Cylinder3D sideEdgeFillet = new Cylinder3D(location, gridLength, filletRadius);
         sideEdgeFillet.appendTranslation(0.0, robotSide.negateIfRightSide(gridWidth / 2 - filletRadius), curbHeight / 2 - filletRadius);
         sideEdgeFillet.appendPitchRotation(Math.PI / 2);
         RigidBodyTransform sideEdgeFilletTransform = new RigidBodyTransform();
         sideEdgeFillet.getPose(sideEdgeFilletTransform);
         combinedTerrainObject.addTerrainObject(new CylinderTerrainObject(sideEdgeFilletTransform, gridLength, filletRadius, edgeAppearance));

         Box3D sideEdge = new Box3D(location, gridLength, curbHeight, curbHeight - filletRadius);
         sideEdge.appendTranslation(0.0, robotSide.negateIfRightSide(gridWidth / 2 - sideEdge.getWidth() / 2), -filletRadius / 2);
         combinedTerrainObject.addRotatableBox(sideEdge, edgeAppearance);

         Box3D sideEdgeTop = new Box3D(location, gridLength, curbHeight - filletRadius, filletRadius);
         sideEdgeTop.appendTranslation(0.0, robotSide.negateIfRightSide(gridWidth / 2 - sideEdgeTop.getWidth() / 2 - filletRadius),
                                       curbHeight / 2 - sideEdgeTop.getHeight() / 2);
         combinedTerrainObject.addRotatableBox(sideEdgeTop, edgeAppearance);
      }

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpInclinedSurface(int startRow, int endRow, int startColumn, int endColumn)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("InclinedSurface");

      AppearanceDefinition curbAppearance = YoAppearance.LightSlateGrey();

      double rampWidth = (endRow - startRow + 1) * gridLength;
      double rampLength = (endColumn - startColumn + 1) * gridWidth;

      double centerX = (getWorldCoordinate(startRow, startColumn).getX() + getWorldCoordinate(endRow, endColumn).getX()) / 2;
      double centerY = (getWorldCoordinate(startRow, startColumn).getY() + getWorldCoordinate(endRow, endColumn).getY()) / 2;
      RotatableRampTerrainObject inclinedSurface = new RotatableRampTerrainObject(centerX, centerY, rampLength, rampWidth, flatGridHeight, -90, curbAppearance);

      // TODO : place grass and stones on this surface.
      combinedTerrainObject.addTerrainObject(inclinedSurface);

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpStormGrate(int row, int column)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("StormGrate");

      AppearanceDefinition crossLineAppearance = YoAppearance.DarkGrey();

      double depthCrossLine = 0.05;
      double thickCrossLine = 0.02;
      int numberOfCrossLines = 15;

      double intervalOfCrossLines = gridLength / (numberOfCrossLines + 1);

      for (int i = 0; i < numberOfCrossLines; i++)
      {
         RigidBodyTransform location = new RigidBodyTransform();
         location.appendTranslation(getWorldCoordinate(row, column));
         location.appendTranslation(gridLength / 2 - intervalOfCrossLines * (i + 1), 0.0, -depthCrossLine / 2);
         Box3D crossLine = new Box3D(location, thickCrossLine, gridWidth, depthCrossLine);

         combinedTerrainObject.addRotatableBox(crossLine, crossLineAppearance);
      }

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpPotholeGrid(int row, int column)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("PotholeGrid");

      return combinedTerrainObject;
   }

   private Point3D getWorldCoordinate(int row, int column)
   {
      double worldX = worldLength / 2 - (row * gridLength + gridLength / 2);
      double worldY = worldWidth / 2 - (column * gridWidth + gridWidth / 2);
      double worldZOnFlatGround = flatGridHeight;
      return new Point3D(worldX, worldY, worldZOnFlatGround);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
