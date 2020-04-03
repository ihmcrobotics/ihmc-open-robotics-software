package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

public class TestEnvironmentTools
{
   public static List<PlanarRegion> createFlatGroundWithWallEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 5.0);
      Point2D wallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform wallTransform = new RigidBodyTransform();
      wallTransform.getTranslation().set(-10.0, 0.0, 0.0);
      wallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion wallRegion = new PlanarRegion(wallTransform,
                                                 new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(wallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithWallOpeningEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 4.5);
      Point2D wallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform leftWallTransform = new RigidBodyTransform();
      leftWallTransform.getTranslation().set(-10.0, 0.5, 0.0);
      leftWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion leftWallRegion = new PlanarRegion(leftWallTransform,
                                                     new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      RigidBodyTransform rightWallTransform = new RigidBodyTransform();
      rightWallTransform.getTranslation().set(-10.0, -5.0, 0.0);
      rightWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion rightWallRegion = new PlanarRegion(rightWallTransform,
                                                      new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(leftWallRegion);
      planarRegions.add(rightWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithWallAwkwardOpeningEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 4.25);
      Point2D wallPointD = new Point2D(0.0, 4.25);

      RigidBodyTransform leftWallTransform = new RigidBodyTransform();
      leftWallTransform.getTranslation().set(-10.0, 0.75, 0.0);
      leftWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion leftWallRegion = new PlanarRegion(leftWallTransform,
                                                     new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      RigidBodyTransform rightWallTransform = new RigidBodyTransform();
      rightWallTransform.getTranslation().set(-10.0, -5.0, 0.0);
      rightWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion rightWallRegion = new PlanarRegion(rightWallTransform,
                                                      new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(leftWallRegion);
      planarRegions.add(rightWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithBoxEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 5.0);
      Point2D frontWallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.getTranslation().set(-11.5, 0.0, 0.0);
      frontWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.getTranslation().set(-8.5, 0.0, 0.0);
      backWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform sideWallTransform = new RigidBodyTransform();
      sideWallTransform.getTranslation().set(-11.5, 0.0, 0.0);
      sideWallTransform.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion sideWallRegion = new PlanarRegion(sideWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(sideWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithBoxInMiddleEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 9.0);
      Point2D frontWallPointD = new Point2D(0.0, 9.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.getTranslation().set(-11.5, -4.5, 0.0);
      frontWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.getTranslation().set(-8.5, -4.5, 0.0);
      backWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWall = new RigidBodyTransform();
      leftSideWall.getTranslation().set(-11.5, 4.5, 0.0);
      leftSideWall.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      RigidBodyTransform rightSideWall = new RigidBodyTransform();
      rightSideWall.getTranslation().set(-11.5, -4.5, 0.0);
      rightSideWall.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion rightSideWallRegion = new PlanarRegion(rightSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(leftSideWallRegion);
      planarRegions.add(rightSideWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithBoxInMiddleEnvironmentButIslandToTheLeft()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up island plane, 20 x 10
      RigidBodyTransform islandTransform = new RigidBodyTransform();
      islandTransform.getTranslation().set(-10.0, 10.2, 0.0);
      PlanarRegion islandPlaneRegion = new PlanarRegion(islandTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 9.0);
      Point2D frontWallPointD = new Point2D(0.0, 9.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.getTranslation().set(-11.5, -4.5, 0.0);
      frontWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.getTranslation().set(-8.5, -4.5, 0.0);
      backWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWall = new RigidBodyTransform();
      leftSideWall.getTranslation().set(-11.5, 4.5, 0.0);
      leftSideWall.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      RigidBodyTransform rightSideWall = new RigidBodyTransform();
      rightSideWall.getTranslation().set(-11.5, -4.5, 0.0);
      rightSideWall.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion rightSideWallRegion = new PlanarRegion(rightSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(islandPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(leftSideWallRegion);
      planarRegions.add(rightSideWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundWithBoxesEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 4.5);
      Point2D frontWallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform frontLeftWallTransform = new RigidBodyTransform();
      frontLeftWallTransform.getTranslation().set(-11.5, 0.5, 0.0);
      frontLeftWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion frontLeftWallRegion = new PlanarRegion(frontLeftWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform frontRightWallTransform = new RigidBodyTransform();
      frontRightWallTransform.getTranslation().set(-11.5, -5.0, 0.0);
      frontRightWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion frontRightWallRegion = new PlanarRegion(frontRightWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      // set up wall, 5x2
      Point2D backWallPointA = new Point2D(2.0, 0.0);
      Point2D backWallPointB = new Point2D(0.0, 0.0);
      Point2D backWallPointC = new Point2D(2.0, 4.25);
      Point2D backWallPointD = new Point2D(0.0, 4.25);

      RigidBodyTransform backLeftWallTransform = new RigidBodyTransform();
      backLeftWallTransform.getTranslation().set(-8.5, 0.75, 0.0);
      backLeftWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion backLeftWallRegion = new PlanarRegion(backLeftWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(backWallPointA, backWallPointB, backWallPointC, backWallPointD)));

      RigidBodyTransform backRightWallTransform = new RigidBodyTransform();
      backRightWallTransform.getTranslation().set(-8.5, -5.0, 0.0);
      backRightWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion backRightWallRegion = new PlanarRegion(backRightWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(backWallPointA, backWallPointB, backWallPointC, backWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWallTransform = new RigidBodyTransform();
      leftSideWallTransform.getTranslation().set(-11.5, 0.625, 0.0);
      leftSideWallTransform.getRotation().setYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontLeftWallRegion);
      planarRegions.add(backLeftWallRegion);
      planarRegions.add(frontRightWallRegion);
      planarRegions.add(backRightWallRegion);
      //      planarRegions.add(leftSideWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createFlatGroundTwoDifferentWidthWallsEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 5.0);
      Point2D wallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform wallTransform = new RigidBodyTransform();
      wallTransform.getTranslation().set(-8.5, 0.0, 0.0);
      wallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion wallRegion = new PlanarRegion(wallTransform,
                                                 new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      Point2D otherWallPointA = new Point2D(2.0, 0.0);
      Point2D otherWallPointB = new Point2D(0.0, 0.0);
      Point2D otherWallPointC = new Point2D(2.0, 4.5);
      Point2D otherWallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform otherWallTransform = new RigidBodyTransform();
      otherWallTransform.getTranslation().set(-11.5, 0.5, 0.0);
      otherWallTransform.getRotation().setToPitchOrientation(-Math.PI / 2.0);
      PlanarRegion otherWallRegion = new PlanarRegion(otherWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(otherWallPointA, otherWallPointB, otherWallPointC, otherWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(wallRegion);
      planarRegions.add(otherWallRegion);

      return planarRegions;
   }

   public static List<PlanarRegion> createCornerEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 10 x 5
      Point2D groundPlanePointA = new Point2D(5.0, -2.5);
      Point2D groundPlanePointB = new Point2D(5.0, 2.5);
      Point2D groundPlanePointC = new Point2D(-5.0, 2.5);
      Point2D groundPlanePointD = new Point2D(-5.0, -2.5);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(0.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      Point2D otherGroundPlanePointA = new Point2D(2.5, 20.0);
      Point2D otherGroundPlanePointB = new Point2D(-2.5, 20.0);
      Point2D otherGroundPlanePointC = new Point2D(-2.5, -20.0);
      Point2D otherGroundPlanePointD = new Point2D(2.5, -20.0);

      RigidBodyTransform groundTransformB = new RigidBodyTransform();
      groundTransformB.getTranslation().set(7.5, 15.0, 0.0);
      PlanarRegion groundPlaneRegionB = new PlanarRegion(groundTransformB, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(otherGroundPlanePointA, otherGroundPlanePointB, otherGroundPlanePointC, otherGroundPlanePointD)));


      planarRegions.add(groundPlaneRegion);
      planarRegions.add(groundPlaneRegionB);

      return planarRegions;
   }

   public static List<PlanarRegion> createSameEdgeRegions()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      double length = 2.0;
      double width = 2.0;

      // set up ground plane, 10 x 5
      Point2D groundPlanePointA = new Point2D(length / 2.0, -width / 2.0);
      Point2D groundPlanePointB = new Point2D(length / 2.0, width / 2.0);
      Point2D groundPlanePointC = new Point2D(-length / 2.0, width / 2.0);
      Point2D groundPlanePointD = new Point2D(-length / 2.0, -width / 2.0);

      RigidBodyTransform groundTransform1 = new RigidBodyTransform();
      groundTransform1.getTranslation().set(0.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion1 = new PlanarRegion(groundTransform1, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      RigidBodyTransform groundTransform2 = new RigidBodyTransform();
      groundTransform2.getTranslation().set(length, 0.0, 0.0);
      PlanarRegion groundPlaneRegion2 = new PlanarRegion(groundTransform2, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      planarRegions.add(groundPlaneRegion1);
      planarRegions.add(groundPlaneRegion2);

      return planarRegions;
   }

   public static List<PlanarRegion> createBigToSmallRegions()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      double length = 10.0;
      double width = 4.0;

      // set up ground plane, 10 x 5
      Point2D groundPlanePoint1A = new Point2D(length / 2.0, -width / 2.0);
      Point2D groundPlanePoint1B = new Point2D(length / 2.0, width / 2.0);
      Point2D groundPlanePoint1C = new Point2D(-length / 2.0, width / 2.0);
      Point2D groundPlanePoint1D = new Point2D(-length / 2.0, -width / 2.0);

      RigidBodyTransform groundTransform1 = new RigidBodyTransform();
      groundTransform1.getTranslation().set(0.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion1 = new PlanarRegion(groundTransform1, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePoint1A, groundPlanePoint1B, groundPlanePoint1C, groundPlanePoint1D)));

      // set up ground plane, 10 x 5
      width = 1.0;
      Point2D groundPlanePoint2A = new Point2D(length / 2.0, -width / 2.0);
      Point2D groundPlanePoint2B = new Point2D(length / 2.0, width / 2.0);
      Point2D groundPlanePoint2C = new Point2D(-length / 2.0, width / 2.0);
      Point2D groundPlanePoint2D = new Point2D(-length / 2.0, -width / 2.0);

      RigidBodyTransform groundTransform2 = new RigidBodyTransform();
      groundTransform2.getTranslation().set(length, 0.0, 0.0);
      PlanarRegion groundPlaneRegion2 = new PlanarRegion(groundTransform2, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePoint2A, groundPlanePoint2B, groundPlanePoint2C, groundPlanePoint2D)));

      planarRegions.add(groundPlaneRegion1);
      planarRegions.add(groundPlaneRegion2);

      return planarRegions;
   }

   public static List<PlanarRegion> createUCornerEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      double width = 1.5;
      // set up ground plane, 10 x 5
      Point2D groundPlanePointA = new Point2D(5.0, -width / 2.0);
      Point2D groundPlanePointB = new Point2D(5.0, width / 2.0);
      Point2D groundPlanePointC = new Point2D(-5.0, width / 2.0);
      Point2D groundPlanePointD = new Point2D(-5.0, -width / 2.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().set(0.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion1 = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      RigidBodyTransform groundTransform2 = new RigidBodyTransform();
      groundTransform2.getTranslation().set(0.0, 10.0, 0.0);
      PlanarRegion groundPlaneRegion2 = new PlanarRegion(groundTransform2, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      Point2D otherGroundPlanePointA = new Point2D(width / 2.0, 5.0 + width / 2.0);
      Point2D otherGroundPlanePointB = new Point2D(-width / 2.0, 5.0 + width / 2.0);
      Point2D otherGroundPlanePointC = new Point2D(-width / 2.0, -(5.0 + width / 2.0));
      Point2D otherGroundPlanePointD = new Point2D(width / 2.0, -(5.0 + width / 2.0));

      RigidBodyTransform groundTransformB = new RigidBodyTransform();
      groundTransformB.getTranslation().set(5.0 + width / 2.0 + 0.05, 5.0, 0.0);
      PlanarRegion groundPlaneRegion3 = new PlanarRegion(groundTransformB, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(otherGroundPlanePointA, otherGroundPlanePointB, otherGroundPlanePointC, otherGroundPlanePointD)));


      planarRegions.add(groundPlaneRegion1);
      planarRegions.add(groundPlaneRegion2);
      planarRegions.add(groundPlaneRegion3);

      return planarRegions;
   }
}
