package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
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
   private static final double palletLength = 1.25;
   private static final double palletWidth = 1.0;

   private static final int gridDimensionX = 3;
   private static final int gridDimensionY = 7;

   private static final double worldLength = gridDimensionX * palletLength;
   private static final double worldWidth = gridDimensionY * palletWidth;

   private static final double tableHeight = 0.5;
   private static final double tableThickness = 0.05;
   private static final double tableLength = 0.7;
   private static final double tableWidth = 0.5;

   private static final double curbHeight = 0.2;

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

      combinedTerrainObject3D.addTerrainObject(setUpPallets("PalletGround"));

      switch (type)
      {
      case A:
         break;
      case B:
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 0));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 0));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 5));
         combinedTerrainObject3D.addTerrainObject(setUpBollard(0, 4, new Point2D(0.4, 0.0), bollardHeight, bollardRadius));
         combinedTerrainObject3D.addTerrainObject(setUpBollard(0, 4, new Point2D(-0.4, 0.0), bollardHeight, bollardRadius));

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

   private CombinedTerrainObject3D setUpPallets(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      AppearanceDefinition palletAppearance = YoAppearance.SandyBrown();
      double palletThickness = 0.1;

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -flatGridHeight - palletThickness / 2));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, worldLength, worldWidth, palletThickness), palletAppearance);
      combinedTerrainObject.addTerrainObject(newBox);

      return combinedTerrainObject;
   }

   private TerrainObject3D setUpFlatGrid(int row, int column)
   {
      AppearanceDefinition gridAppearance = YoAppearance.Grey();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column).getX(), getWorldCoordinate(row, column).getY(), -flatGridHeight / 2);

      return new RotatableBoxTerrainObject(new Box3D(location, palletLength, palletWidth, flatGridHeight), gridAppearance);
   }

   private TerrainObject3D setUpBollard(int row, int column, Point2D positionInGrid, double height, double radius)
   {
      AppearanceDefinition bollardAppearance = YoAppearance.Yellow();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column).getX(), getWorldCoordinate(row, column).getY(), 0.0);
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), height / 2);

      return new CylinderTerrainObject(location, height, radius, bollardAppearance);
   }

   private TerrainObject3D setUpTable(int row, int column, Point2D positionInGrid)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column).getX(), getWorldCoordinate(row, column).getY(), 0.0);
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), 0.0);

      double xStart = location.getTranslationX() - tableLength / 2;
      double yStart = location.getTranslationY() - tableWidth / 2;
      double xEnd = location.getTranslationX() + tableLength / 2;
      double yEnd = location.getTranslationY() + tableWidth / 2;
      double zStart = tableHeight - tableThickness;
      double zEnd = tableHeight;

      return new SimpleTableTerrainObject(xStart, yStart, xEnd, yEnd, zStart, zEnd);
   }

   private CombinedTerrainObject3D setUpCurb(int row, int column)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("CurbGrid");

      combinedTerrainObject.addTerrainObject(setUpFlatGrid(row, column));

      AppearanceDefinition curbAppearance = YoAppearance.Chocolate();
      AppearanceDefinition edgeAppearance = YoAppearance.DarkGrey();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column).getX(), getWorldCoordinate(row, column).getY(), 0.0);
      location.appendTranslation(0.0, 0.0, curbHeight / 2);

      combinedTerrainObject.addRotatableBox(location, palletLength, palletWidth - curbHeight * 2, curbHeight, curbAppearance);

      for (RobotSide robotSide : RobotSide.values)
      {
         Box3D sideEdge = new Box3D(location, palletLength, curbHeight, curbHeight);
         sideEdge.appendTranslation(0.0, robotSide.negateIfRightSide(palletWidth / 2 - curbHeight / 2), 0.0);
         combinedTerrainObject.addRotatableBox(sideEdge, edgeAppearance);
      }

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpPotholeGrid(int row, int column)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("PotholeGrid");

      return combinedTerrainObject;
   }

   private Point2D getWorldCoordinate(int row, int column)
   {
      return new Point2D(worldLength / 2 - (row * palletLength + palletLength / 2), worldWidth / 2 - (column * palletWidth + palletWidth / 2));
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
