package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.PlanarRegionTerrainObject;
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

   private static final CourseType type = CourseType.A;

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
   private static final double bumpRun = 0.3;
   private static final double bumpWidth = gridLength;
   private static final double bumpSideMargin = 0.01;

   private static final double bollardHeight = 0.7;
   private static final double bollardRadius = 0.075;

   private static final double potholeDepth = 0.15;

   public CTTSOObstacleCourseEnvironment()
   {
      combinedTerrainObject3D = new CombinedTerrainObject3D("CTTSOObstacleCourseEnvironment");

      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      switch (type)
      {
      case A:
         combinedTerrainObject3D.addTerrainObject(setUpInclinedSurface(0, 0, 0, 3));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 0));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 0));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 1));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 1));
         combinedTerrainObject3D.addTerrainObject(createBump(1, 1, new Point2D(), bumpHeight, bumpRun, bumpWidth, 0.0));
         combinedTerrainObject3D.addTerrainObject(createBump(2, 1, new Point2D(), bumpHeight, bumpRun, bumpWidth, 0.0));

         //         combinedTerrainObject3D.addTerrainObject(setUpPotholeGrid(1, 2));
         combinedTerrainObject3D.addTerrainObject(setUpPotholeGrid(1, 2, new Point2D(0.2, 0.0), 0.3, 0.05));
         //combinedTerrainObject3D.addTerrainObject(setUpPotholeGrid(2, 2, 1, 0.1, 0.1));

         combinedTerrainObject3D.addTerrainObject(setUpCurb(1, 3));
         combinedTerrainObject3D.addTerrainObject(setUpCurb(2, 3));
         combinedTerrainObject3D.addTerrainObject(createTable(1, 3, new Point2D(0.1, 0.0)));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 4));
         combinedTerrainObject3D.addTerrainObject(createBollard(1, 4, new Point2D(0.0, 0.0), bollardHeight, bollardRadius));
         combinedTerrainObject3D.addTerrainObject(createBollard(2, 4, new Point2D(0.4, 0.0), bollardHeight, bollardRadius));
         combinedTerrainObject3D.addTerrainObject(createBollard(2, 4, new Point2D(-0.4, 0.0), bollardHeight, bollardRadius));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 5));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 5));
         combinedTerrainObject3D.addTerrainObject(setUpStormGrate(2, 5));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 6));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 6));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 6));

         break;
      case B:
         combinedTerrainObject3D.addTerrainObject(setUpInclinedSurface(0, 0, 0, 3));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 0));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 0));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 4));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(0, 5));
         combinedTerrainObject3D.addTerrainObject(createBollard(0, 4, new Point2D(0.4, 0.0), bollardHeight, bollardRadius));
         combinedTerrainObject3D.addTerrainObject(createBollard(0, 4, new Point2D(-0.4, 0.0), bollardHeight, bollardRadius));

         combinedTerrainObject3D.addTerrainObject(setUpStormGrate(1, 5));

         //combinedTerrainObject3D.addTerrainObject(setUpPotholeGrid(1, 1));
         //combinedTerrainObject3D.addTerrainObject(setUpPotholeGrid(2, 1));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(1, 2));
         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 2));

         combinedTerrainObject3D.addTerrainObject(setUpCurb(1, 3));
         combinedTerrainObject3D.addTerrainObject(setUpCurb(2, 3));

         combinedTerrainObject3D.addTerrainObject(setUpFlatGrid(2, 6));
         combinedTerrainObject3D.addTerrainObject(createTable(1, 6, new Point2D(-0.1, 0.0)));

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

   private TerrainObject3D createBollard(int row, int column, Point2D positionInGrid, double height, double radius)
   {
      AppearanceDefinition bollardAppearance = YoAppearance.Yellow();

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), height / 2);

      return new CylinderTerrainObject(location, height, radius, bollardAppearance);
   }

   private TerrainObject3D createTable(int row, int column, Point2D positionInGrid)
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

   private TerrainObject3D createBump(int row, int column, Point2D positionInGrid, double height, double run, double width, double rotateYaw)
   {
      AppearanceDefinition bumpAppearance = YoAppearance.Yellow();

      double radius = height / 2 + run * run / 8 / height;
      double alpha = Math.asin(run / 2 / radius);
      double depth = radius * Math.cos(alpha);

      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(getWorldCoordinate(row, column));
      location.appendTranslation(positionInGrid.getX(), positionInGrid.getY(), -depth);
      location.appendYawRotation(rotateYaw);
      location.appendPitchRotation(Math.PI / 2);

      return new CylinderTerrainObject(location, width - bumpSideMargin, radius, bumpAppearance);
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
      AppearanceDefinition wallAppearance = YoAppearance.BurlyWood();
      YoAppearance.makeTransparent(wallAppearance, 0.7f);

      double wallThickness = 0.05;
      double wallHeight = 1.0;

      double rampWidth = (endRow - startRow + 1) * gridLength - wallThickness;
      double rampLength = (endColumn - startColumn + 1) * gridWidth;

      double centerX = (getWorldCoordinate(startRow, startColumn).getX() + getWorldCoordinate(endRow, endColumn).getX()) / 2 + wallThickness / 2;
      double centerY = (getWorldCoordinate(startRow, startColumn).getY() + getWorldCoordinate(endRow, endColumn).getY()) / 2;
      RotatableRampTerrainObject inclinedSurface = new RotatableRampTerrainObject(centerX, centerY, rampLength, rampWidth, flatGridHeight, -90, curbAppearance);

      // TODO : place grass and stones on this surface.
      combinedTerrainObject.addTerrainObject(inclinedSurface);

      RigidBodyTransform wallLocation = new RigidBodyTransform();
      wallLocation.appendTranslation(centerX, centerY, 0.0);
      wallLocation.appendTranslation(-wallThickness / 2 - rampWidth / 2, 0.0, wallHeight / 2);

      combinedTerrainObject.addTerrainObject(new RotatableBoxTerrainObject(wallLocation, wallThickness, rampLength, wallHeight, wallAppearance));

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

   private static final int POINTS_PER_POTHOLE = 8;
   private static final double allowablePenetrationThickness = 0.07;

   private CombinedTerrainObject3D setUpPotholeGrid(int row, int column, Point2DReadOnly location, double centerToPoints, double depthToCentroid)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("PotholeGrid");

      AppearanceDefinition potholeAppearance = YoAppearance.DarkGrey();
      AppearanceDefinition flatAppearance = YoAppearance.Grey();

      RigidBodyTransform centroidTransform = new RigidBodyTransform();
      centroidTransform.appendTranslation(getWorldCoordinate(row, column));
      centroidTransform.appendTranslation(location.getX(), location.getY(), 0.0);
      RegularPolygon potholePolygon = new RegularPolygon(centerToPoints, POINTS_PER_POTHOLE);

      PotholePlanarRegionProvider potholePlanarRegionProvider = new PotholePlanarRegionProvider(potholePolygon, centroidTransform.getTranslationVector(),
                                                                                                depthToCentroid);
      List<PlanarRegion> potholePlanarRegions = potholePlanarRegionProvider.getPlanarRegions();
      for (int i = 0; i < potholePlanarRegions.size(); i++)
      {
         PlanarRegionTerrainObject potholeTerrainObject = new PlanarRegionTerrainObject(potholePlanarRegions.get(i), allowablePenetrationThickness,
                                                                                        potholeAppearance);
         combinedTerrainObject.addTerrainObject(potholeTerrainObject);
      }

      ConvexPolygon2D flatRegionPolygon = new ConvexPolygon2D();
      flatRegionPolygon.addVertex(gridLength / 2 - location.getX(), gridWidth / 2 - location.getY());
      flatRegionPolygon.addVertex(gridLength / 2 - location.getX(), -gridWidth / 2 - location.getY());
      flatRegionPolygon.addVertex(-gridLength / 2 - location.getX(), -gridWidth / 2 - location.getY());
      flatRegionPolygon.addVertex(-gridLength / 2 - location.getX(), gridWidth / 2 - location.getY());
      flatRegionPolygon.update();
      List<PlanarRegion> flatPlanarRegions = potholePlanarRegionProvider.getAdjustedFlatPlanarRegions(flatRegionPolygon);

      for (int i = 0; i < flatPlanarRegions.size(); i++)
      {
         PlanarRegionTerrainObject potholeTerrainObject = new PlanarRegionTerrainObject(flatPlanarRegions.get(i), allowablePenetrationThickness,
                                                                                        flatAppearance);
         combinedTerrainObject.addTerrainObject(potholeTerrainObject);
      }

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpPotholeGrid(int row, int column, int numberOfPotholes, double centerToPoints, double depthToCentroid)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("PotholeGrid");

      AppearanceDefinition potholeAppearance = YoAppearance.DarkGrey();
      AppearanceDefinition flatAppearance = YoAppearance.Grey();

      for (int i = 0; i < numberOfPotholes; i++)
      {
         double intervalX = gridLength / numberOfPotholes;
         Point2D location = new Point2D(gridLength / 2 - intervalX * (i + 0.5), 0.0);

         RigidBodyTransform centroidTransform = new RigidBodyTransform();
         centroidTransform.appendTranslation(getWorldCoordinate(row, column));
         centroidTransform.appendTranslation(location.getX(), location.getY(), 0.0);
         RegularPolygon potholePolygon = new RegularPolygon(centerToPoints, POINTS_PER_POTHOLE);

         PotholePlanarRegionProvider potholePlanarRegionProvider = new PotholePlanarRegionProvider(potholePolygon, centroidTransform.getTranslationVector(),
                                                                                                   depthToCentroid);
         List<PlanarRegion> potholePlanarRegions = potholePlanarRegionProvider.getPlanarRegions();
         for (int j = 0; j < potholePlanarRegions.size(); j++)
         {
            PlanarRegionTerrainObject potholeTerrainObject = new PlanarRegionTerrainObject(potholePlanarRegions.get(j), allowablePenetrationThickness,
                                                                                           potholeAppearance);
            combinedTerrainObject.addTerrainObject(potholeTerrainObject);
         }

         ConvexPolygon2D flatRegionPolygon = new ConvexPolygon2D();
         flatRegionPolygon.addVertex(intervalX / 2 - location.getX(), gridWidth / 2 - location.getY());
         flatRegionPolygon.addVertex(intervalX / 2 - location.getX(), -gridWidth / 2 - location.getY());
         flatRegionPolygon.addVertex(-intervalX / 2 - location.getX(), -gridWidth / 2 - location.getY());
         flatRegionPolygon.addVertex(-intervalX / 2 - location.getX(), gridWidth / 2 - location.getY());
         flatRegionPolygon.update();
         List<PlanarRegion> flatPlanarRegions = potholePlanarRegionProvider.getAdjustedFlatPlanarRegions(flatRegionPolygon);

         for (int j = 0; j < flatPlanarRegions.size(); j++)
         {
            PlanarRegionTerrainObject potholeTerrainObject = new PlanarRegionTerrainObject(flatPlanarRegions.get(j), allowablePenetrationThickness,
                                                                                           flatAppearance);
            combinedTerrainObject.addTerrainObject(potholeTerrainObject);
         }
      }

      return combinedTerrainObject;
   }

   private class RegularPolygon
   {
      private final double centerToPoints;
      private final ConvexPolygon2D polygon;

      private RegularPolygon(double centerToPoints, int numberOfPoints)
      {
         this.centerToPoints = centerToPoints;

         polygon = new ConvexPolygon2D();
         for (int i = 0; i < numberOfPoints; i++)
         {
            double intervalAngle = 2 * Math.PI / numberOfPoints;
            RigidBodyTransform vertexTransform = new RigidBodyTransform();
            vertexTransform.appendYawRotation(intervalAngle * i);
            vertexTransform.appendTranslation(centerToPoints, 0.0, 0.0);
            polygon.addVertex(vertexTransform.getTranslationX(), vertexTransform.getTranslationY());
         }
         polygon.update();
      }

      double getCenterToPoints()
      {
         return centerToPoints;
      }

      ConvexPolygon2D getPolygon()
      {
         return polygon;
      }
   }

   private class PotholePlanarRegionProvider
   {
      private final static double defaultRatio = 0.5;
      private final Point3D centroidLocation;
      private final double depthToCentroid;
      private final double ratioToInnerPolygon;
      private final RegularPolygon potholeRegularPolygon;

      private PotholePlanarRegionProvider(RegularPolygon regularPolygon, Vector3DReadOnly centroidLocation, double depthToCentroid)
      {
         this(regularPolygon, centroidLocation, depthToCentroid, defaultRatio);
      }

      private PotholePlanarRegionProvider(RegularPolygon regularPolygon, Vector3DReadOnly centroidLocation, double depthToCentroid, double ratioToInner)
      {
         this.potholeRegularPolygon = regularPolygon;
         this.centroidLocation = new Point3D(centroidLocation);
         this.depthToCentroid = depthToCentroid;
         this.ratioToInnerPolygon = ratioToInner;
      }

      /*
       * flatRegionPolygon will be divided into 2 pieces.
       */
      public List<PlanarRegion> getAdjustedFlatPlanarRegions(ConvexPolygon2D flatRegionPolygon)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.appendTranslation(centroidLocation);

         List<PlanarRegion> planarRegions = new ArrayList<PlanarRegion>();
         ConvexPolygon2D polygon = potholeRegularPolygon.getPolygon();
         int numberOfVertices = polygon.getNumberOfVertices();
         int numberOfOuterVertices = flatRegionPolygon.getNumberOfVertices();

         List<LineSegment2D> outerPolygonLines = new ArrayList<LineSegment2D>();
         for (int i = 0; i < numberOfOuterVertices - 1; i++)
            outerPolygonLines.add(new LineSegment2D(flatRegionPolygon.getVertex(i), flatRegionPolygon.getVertex(i + 1)));
         outerPolygonLines.add(new LineSegment2D(flatRegionPolygon.getVertex(numberOfOuterVertices - 1), flatRegionPolygon.getVertex(0)));

         List<LineSegment2D> potholePolygonEdges = new ArrayList<LineSegment2D>();
         for (int i = 0; i < numberOfVertices - 1; i++)
            potholePolygonEdges.add(new LineSegment2D(polygon.getVertex(i), polygon.getVertex(i + 1)));
         potholePolygonEdges.add(new LineSegment2D(polygon.getVertex(numberOfVertices - 1), polygon.getVertex(0)));

         for (int i = 0; i < potholePolygonEdges.size(); i++)
            planarRegions.add(new PlanarRegion(transform, getOuterPolygon(potholePolygonEdges.get(i), outerPolygonLines)));

         for (int i = 0; i < numberOfOuterVertices; i++)
         {
            double min = Double.MAX_VALUE;
            int indexOfClosestPotholeVertex = 0;
            for (int j = 0; j < numberOfVertices; j++)
            {
               double distance = flatRegionPolygon.getVertex(i).distance(polygon.getVertex(j));
               if (distance < min)
               {
                  min = distance;
                  indexOfClosestPotholeVertex = j;
               }
            }
            Point2DReadOnly closestPotholeVertex = polygon.getVertex(indexOfClosestPotholeVertex);
            Point2DReadOnly outerVertex = flatRegionPolygon.getVertex(i);
            LineSegment2D rightLine = outerPolygonLines.get(i);
            LineSegment2D leftLine;
            if (i == 0)
               leftLine = outerPolygonLines.get(numberOfOuterVertices - 1);
            else
               leftLine = outerPolygonLines.get(i - 1);
            
            //planarRegions.add(new PlanarRegion(transform, getOuterPolygon(leftLine, rightLine, outerVertex, closestPotholeVertex)));
         }
         
         return planarRegions;
      }

      private ConvexPolygon2D getOuterPolygon(LineSegment2D polygonEdge, List<LineSegment2D> outerPolygonLines)
      {
         ConvexPolygon2D outerPolygon = new ConvexPolygon2D();

         outerPolygon.addVertex(polygonEdge.getFirstEndpoint());
         outerPolygon.addVertex(polygonEdge.getSecondEndpoint());

         double minQuery = Double.MAX_VALUE;
         int indexOfClosestOuterLine = 0;
         for (int j = 0; j < outerPolygonLines.size(); j++)
         {
            LineSegment2D outerLine = outerPolygonLines.get(j);
            double query = outerLine.distance(polygonEdge.getFirstEndpoint()) + outerLine.distance(polygonEdge.getSecondEndpoint());

            if (query < minQuery)
            {
               minQuery = query;
               indexOfClosestOuterLine = j;
            }
         }
         LineSegment2D closestOuterLine = outerPolygonLines.get(indexOfClosestOuterLine);
         PrintTools.info(""+indexOfClosestOuterLine);

         Point2D pointOnOuterLineOne = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(polygonEdge.getFirstEndpoint(),
                                                                                               closestOuterLine.getFirstEndpoint(),
                                                                                               closestOuterLine.getSecondEndpoint());
         Point2D pointOnOuterLineTwo = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(polygonEdge.getSecondEndpoint(),
                                                                                               closestOuterLine.getFirstEndpoint(),
                                                                                               closestOuterLine.getSecondEndpoint());

         outerPolygon.addVertex(pointOnOuterLineOne);
         outerPolygon.addVertex(pointOnOuterLineTwo);

         outerPolygon.update();

         return outerPolygon;
      }

      private ConvexPolygon2D getOuterPolygon(LineSegment2D leftLine, LineSegment2D rightLine, Point2DReadOnly vertex, Point2DReadOnly point)
      {
         ConvexPolygon2D outerPolygon = new ConvexPolygon2D();

         outerPolygon.addVertex(vertex);
         outerPolygon.addVertex(point);

         Point2D pointOnOuterLineOne = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point, leftLine.getFirstEndpoint(),
                                                                                               leftLine.getSecondEndpoint());
         Point2D pointOnOuterLineTwo = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point, rightLine.getFirstEndpoint(),
                                                                                               rightLine.getSecondEndpoint());

         outerPolygon.addVertex(pointOnOuterLineOne);
         outerPolygon.addVertex(pointOnOuterLineTwo);

         outerPolygon.update();

         return outerPolygon;
      }

      /*
       * this method returns planar regions including bottom region and
       * trapezoid shaped side region.
       */
      public List<PlanarRegion> getPlanarRegions()
      {
         List<PlanarRegion> planarRegions = new ArrayList<PlanarRegion>();
         ConvexPolygon2D polygon = potholeRegularPolygon.getPolygon();
         int numberOfVertices = polygon.getNumberOfVertices();

         // Bottom Region.
         RigidBodyTransform bottomTransform = new RigidBodyTransform();
         bottomTransform.appendTranslation(centroidLocation);
         bottomTransform.appendTranslation(0.0, 0.0, -depthToCentroid);

         ConvexPolygon2D bottomPolygon = new ConvexPolygon2D();
         for (int i = 0; i < numberOfVertices; i++)
         {
            Point2D vertex = new Point2D(potholeRegularPolygon.getPolygon().getVertex(i));
            vertex.scale(ratioToInnerPolygon);
            bottomPolygon.addVertex(vertex);
         }
         bottomPolygon.update();

         PlanarRegion bottomPlanarRegion = new PlanarRegion(bottomTransform, bottomPolygon);
         //planarRegions.add(bottomPlanarRegion);

         // Side Regions (trapezoids).
         for (int i = 0; i < numberOfVertices; i++)
         {
            RigidBodyTransform sideTransform = new RigidBodyTransform();
            sideTransform.appendTranslation(centroidLocation);
            double intervalAngle = 2 * Math.PI / numberOfVertices;
            double yawAngle = i * intervalAngle + intervalAngle / 2;
            double centerToSideLine = potholeRegularPolygon.getCenterToPoints() * Math.cos(intervalAngle / 2);
            double translationX = centerToSideLine * (1 + ratioToInnerPolygon) / 2;
            double pitchAngle = -Math.atan(depthToCentroid / (centerToSideLine * (1 - ratioToInnerPolygon)));
            sideTransform.appendYawRotation(yawAngle);
            sideTransform.appendTranslation(translationX, 0.0, -depthToCentroid / 2);
            sideTransform.appendPitchRotation(pitchAngle);

            double trapezoidHeight = depthToCentroid / Math.sin(-pitchAngle);
            double trapezoidUpperLine = 2 * potholeRegularPolygon.getCenterToPoints() * Math.sin(intervalAngle / 2);
            double trapezoidLowerLine = trapezoidUpperLine * ratioToInnerPolygon;

            ConvexPolygon2D trapezoidPolygon = new ConvexPolygon2D();
            trapezoidPolygon.addVertex(trapezoidHeight / 2, trapezoidUpperLine / 2);
            trapezoidPolygon.addVertex(-trapezoidHeight / 2, trapezoidLowerLine / 2);
            trapezoidPolygon.addVertex(-trapezoidHeight / 2, -trapezoidLowerLine / 2);
            trapezoidPolygon.addVertex(trapezoidHeight / 2, -trapezoidUpperLine / 2);
            trapezoidPolygon.update();

            PlanarRegion sidePlanarRegion = new PlanarRegion(sideTransform, trapezoidPolygon);
            planarRegions.add(sidePlanarRegion);
         }

         return planarRegions;
      }
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
