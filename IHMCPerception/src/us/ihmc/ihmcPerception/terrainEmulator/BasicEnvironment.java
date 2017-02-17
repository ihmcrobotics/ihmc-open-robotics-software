package us.ihmc.ihmcPerception.terrainEmulator;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TrussWithSimpleCollisions;

/**
 * Created by agrabertilton on 1/12/15.
 */
public class BasicEnvironment{
      private final CombinedTerrainObject3D combinedTerrainObject3D;

      private final Random random = new Random(1989L);

      private static final double ROCKS_START_Y = 3.5;
      private static final double ROCK_PATH_LENGTH = 8.0;

      private static final int NUM_ROCKS = 80;
      private static final double MAX_ROCK_CENTROID_HEIGHT = 0.2;
      private static final int POINTS_PER_ROCK = 21;

      // chance unevenness of rocks
      private static final double MAX_ABS_XY_NORMAL_VALUE = 0.0;
      private static final double ROCK_FIELD_WIDTH = 2.0;
      private static final double ROCK_BOUNDING_BOX_WIDTH = 0.5;

      // FULLY_RANDOM Will do a neat grid if set to false;
      private static final boolean FULLY_RANDOM = true;
      private static final int ROCKS_PER_ROW = 4;

      // for path 8, if DIFFICULT_STEPPING_STONES true creates an extension to the path with harder steps
      private static final boolean DIFFICULT_STEPPING_STONES = false;

      private static final AppearanceDefinition cinderBlockAppearance = YoAppearance.DarkGray();
      private static final double cinderBlockLength = 0.40;    // 40 cm (approx 16 in, just less than 16in)
      private static final double cinderBlockWidth = cinderBlockLength / 2.0;
      private static final double cinderBlockHeight = 0.15;    // 15 cm (approx 6 in, less than 6 in, but consistent with other cm measurements)
      private static final double overlapToPreventGaps = 0.002;

      private static final double cinderBlockTiltDegrees = 15;
      private static final double cinderBlockTiltRadians = Math.toRadians(cinderBlockTiltDegrees);

      private static final boolean VISUALIZE_BOUNDING_BOXES = false;

      private static final boolean SHOW_FULL_TESTBED = false;

      enum BLOCKTYPE {FLAT, FLATSKEW, UPRIGHTSKEW, ANGLED}

      ;

      private boolean addLimboBar = false;

      // private static final double FLOOR_THICKNESS = 0.001;

      public BasicEnvironment()
      {
         combinedTerrainObject3D = new CombinedTerrainObject3D("BasicEnvironment");

         // addCalibrationCube();
         combinedTerrainObject3D.addTerrainObject(setUpPath1Rocks3D("Path1 Rocks"));

         // setUpPath2SmallCones(combinedTerrainObject);
         combinedTerrainObject3D.addTerrainObject(setUpPath3RampsWithLargeBlocks3D("Path3 Ramps With Large Blocks"));
         combinedTerrainObject3D.addTerrainObject(setUpPath4DRCTrialsTrainingWalkingCourse("Path 4 Walking Course"));

         // combinedTerrainObject.addTerrainObject(setUpPath4DRCTrialsTrainingWalkingCourseDifficult());
         combinedTerrainObject3D.addTerrainObject(setUpPathDRCTrialsSteps("Ladder"));
         combinedTerrainObject3D.addTerrainObject(setUpTrialsQuals("Quals"));

         combinedTerrainObject3D.addTerrainObject(setUpPath5NarrowDoor("Path5 Narrow Door"));
         combinedTerrainObject3D.addTerrainObject(setUpPath6Barriers("Path6 Barriers"));
         combinedTerrainObject3D.addTerrainObject(setUpPath7Stairs("Path7 Stairs"));
         combinedTerrainObject3D.addTerrainObject(setUpPath8RampsWithSteppingStones("Path8 Ramps with Stepping Stones"));

         combinedTerrainObject3D.addTerrainObject(setUpGround("Ground"));

         if (addLimboBar)
            addLimboBar(combinedTerrainObject3D);

         // testRotatableRampsSetupForGraphicsAndCollision();
         // addFalseStair();

         if (VISUALIZE_BOUNDING_BOXES)
         {
            StringBuffer stringBuffer = new StringBuffer();
            combinedTerrainObject3D.recursivelyPrintBoundingBoxes(stringBuffer);
            System.out.println(stringBuffer);

            combinedTerrainObject3D.recursivelyAddBoundingBoxVisualizerToLinkGraphics(YoAppearance.Purple());
         }
      }

      public void addCalibrationCube()
      {
         AppearanceDefinition app = YoAppearance.Beige();
         setUpSlopedBox(combinedTerrainObject3D, 2, 2, .25, .5, .5, .5, 0, 0, app);
      }

      private void addFalseStair()
      {
         final double courseAngle = 3 * 45. / 2;
         final double startDistance = 6.75;
         AppearanceDefinition app = YoAppearance.Red();
         YoAppearance.makeTransparent(app, 1);

         final double stepWidth = 0.812;
         final double stepTread = stepWidth;
         final double stepThickness = 0.0381;
         final double stepRise = 0.3048;

         double[] centerPointLocal = {startDistance + stepTread / 2, 0};
         double[] centerPoint;
         double stairTopHeight = stepRise;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpFloatingStair(combinedTerrainObject3D, centerPoint, stepWidth, stepTread, stepThickness, stairTopHeight, courseAngle, app);
      }

      private static CombinedTerrainObject3D setUpPathDRCTrialsSteps(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         final double courseAngle = 3 * 45. / 2;
         final double courseStartDistance = 7;
         AppearanceDefinition app = YoAppearance.BlackMetalMaterial();

         final double stepTread = 0.2794;
         final double stepWidth = 1.06;
         final double stepThickness = 0.1778;
         final double stepRise = 0.1778;
         final double stepRun = 0.2794;
         final int nStairs = 3;

         final double landingRun = 0.61;
         final double stairSupportThickness = 0.0508;
         final double stairSupportWidth = 0.132;

         final double stairSlope = Math.atan(stepRise / stepRun);

         // steps
         double[] centerPointLocal = {courseStartDistance + stepTread / 2, 0};
         double[] centerPoint;
         double stairTopHeight = 0;
         for (int i = 0; i < nStairs - 1; i++)
         {
            centerPointLocal[0] += stepRun;
            stairTopHeight += stepRise;
            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
            setUpFloatingStair(combinedTerrainObject, centerPoint, stepWidth, stepTread, stepThickness, stairTopHeight, courseAngle, app);
         }

         centerPointLocal[0] += stepRun - stepTread / 2 + landingRun / 2;
         double topLandingCenter = centerPointLocal[0];
         stairTopHeight += stepRise;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpFloatingStair(combinedTerrainObject, centerPoint, stepWidth, landingRun, stepThickness, stairTopHeight, courseAngle, app);

         // side supports
         double sinStairSlope = Math.sin(stairSlope);
         double cosStairSlope = Math.cos(stairSlope);
         double supportLength = stairTopHeight / sinStairSlope;
         double distanceFromLowerSupportCornerToGroundStepLowerBackCorner = (stairSupportWidth * cosStairSlope - stepThickness) / sinStairSlope;
         double distanceFromSupportGroundCornerToLeadingGroundStepEdge = stairSupportWidth * sinStairSlope
               + distanceFromLowerSupportCornerToGroundStepLowerBackCorner * cosStairSlope - stepTread;
         centerPointLocal[0] = courseStartDistance + supportLength / 2 * cosStairSlope + stairSupportWidth / 2 * sinStairSlope
               - distanceFromSupportGroundCornerToLeadingGroundStepEdge;
         double zCenter = supportLength / 2 * sinStairSlope - stairSupportWidth / 2 * cosStairSlope;
         centerPointLocal[1] = stepWidth / 2 + stairSupportThickness / 2;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, supportLength, stairSupportThickness, stairSupportWidth, stairSlope,
               courseAngle, app);

         centerPointLocal[1] = -centerPointLocal[1];
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, supportLength, stairSupportThickness, stairSupportWidth, stairSlope,
               courseAngle, app);

         double topSupportLength = landingRun + distanceFromSupportGroundCornerToLeadingGroundStepEdge;
         centerPointLocal[0] = topLandingCenter - distanceFromSupportGroundCornerToLeadingGroundStepEdge / 2;
         zCenter = stairTopHeight - stairSupportWidth / 2;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, topSupportLength, stairSupportThickness, stairSupportWidth, 0,
               courseAngle, app);

         centerPointLocal[1] = -centerPointLocal[1];
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, topSupportLength, stairSupportThickness, stairSupportWidth, 0,
               courseAngle, app);


         final double railingDiameter = stairSupportThickness;
         final double topRailingHeight = 1.067;
         final int nTopRailingCrossBars = 2;
         final double stairRailSupportLength = 0.5715;
         final int nStairRailSupports = 2;
         final boolean extendRailsToGround = true;
         final double stairRailSupportStartHeight = railingDiameter;
         final double stairRailSupportEndHeight = 2.438;

         double railingSupportAngle = stairSlope + Math.PI / 2;
         double xCenterOffset = stairRailSupportLength / 2 * Math.cos(railingSupportAngle);
         double zCenterOffset = stairRailSupportLength / 2 * Math.sin(railingSupportAngle);

         for (int ySign = -1; ySign <= 1; ySign += 2)
         {
            centerPointLocal[1] = ySign * (stepWidth / 2 + railingDiameter / 2);

            for (int xSign = -1; xSign <= 2; xSign += 2)
            {
               // vertical supports for railing on top landing
               centerPointLocal[0] = topLandingCenter + xSign * (landingRun / 2 - railingDiameter / 2);
               centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
               setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], stairTopHeight + topRailingHeight / 2, topRailingHeight,
                     railingDiameter / 2, Math.PI / 2, 0, app);
            }

            // horizontal railing on top landing
            centerPointLocal[0] = topLandingCenter;
            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);

            for (int n = 0; n < nTopRailingCrossBars; n++)
            {
               setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], stairTopHeight + topRailingHeight / (nTopRailingCrossBars) * (n + 1),
                     landingRun, railingDiameter / 2, 0, courseAngle, app);
            }

            // stairs railing supports
            for (int n = 0; n < nStairRailSupports; n++)
            {
               double zBase = stairRailSupportStartHeight + n * (stairRailSupportEndHeight - stairRailSupportStartHeight) / (nStairRailSupports - 1);
               double xBase = zBase / Math.tan(stairSlope) - distanceFromSupportGroundCornerToLeadingGroundStepEdge + courseStartDistance;
               centerPointLocal[0] = xBase + xCenterOffset;
               zCenter = zBase + zCenterOffset;
               centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
               setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, stairRailSupportLength, railingDiameter / 2,
                     railingSupportAngle, courseAngle, app);
            }

            // stair railing
            double x0Railing = centerPointLocal[0] + xCenterOffset;
            double z0Railing = zCenter + zCenterOffset;
            double vxRailing = cosStairSlope;
            double vzRailing = sinStairSlope;

            double xEnd = topLandingCenter - (landingRun / 2 - railingDiameter / 2);
            double zEnd = z0Railing + vzRailing * (xEnd - x0Railing) / vxRailing;
            if (zEnd > stairTopHeight + topRailingHeight)
            {
               zEnd = stairTopHeight + topRailingHeight;
               xEnd = x0Railing + vxRailing * (zEnd - z0Railing) / vzRailing;

               // Extend top rail
               centerPointLocal[0] = (topLandingCenter - landingRun / 2 + xEnd) / 2;
               centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
               setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], zEnd, topLandingCenter - landingRun / 2 - xEnd, railingDiameter / 2, 0,
                     courseAngle, app);
            }

            double zStart;
            double xStart;
            if (extendRailsToGround)
            {
               zStart = 0;
               xStart = x0Railing + vxRailing * (zStart - z0Railing) / vzRailing;
            }
            else
            {
               zStart = stairRailSupportStartHeight + 2 * zCenterOffset;
               xStart = stairRailSupportStartHeight / Math.tan(stairSlope) - distanceFromSupportGroundCornerToLeadingGroundStepEdge + courseStartDistance;

               zStart -= railingDiameter / 2 * sinStairSlope;
               xStart -= railingDiameter / 2 * cosStairSlope;
            }

            centerPointLocal[0] = (xStart + xEnd) / 2;
            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
            setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], (zStart + zEnd) / 2,
                  Math.sqrt((xEnd - xStart) * (xEnd - xStart) + (zEnd - zStart) * (zEnd - zStart)), railingDiameter / 2, stairSlope, courseAngle,
                  app);

         }


         return combinedTerrainObject;
      }

      private TerrainObject3D setUpPath1Rocks3D(String name)
      {
         return addRocks3D(name);
      }

      private void setUpPath2SmallCones()
      {
         int numCones = 4;
         float initialOffset = 2.952f;
         float coneSeparation = 1.5f;
         float coneColorSeparateion = 0.1f;

         for (int i = 0; i < numCones; i++)
         {
            AppearanceDefinition cone1;
            AppearanceDefinition cone2;
            if (i % 2 == 0)
            {
               cone1 = YoAppearance.Green();
               cone2 = YoAppearance.Red();
            }
            else
            {
               cone1 = YoAppearance.Red();
               cone2 = YoAppearance.Green();
            }

            setUpCone(combinedTerrainObject3D, initialOffset + (i * coneSeparation) + coneColorSeparateion, -(initialOffset + (i * coneSeparation)), .25, .25,
                  0.5, cone1);
            setUpCone(combinedTerrainObject3D, initialOffset + (i * coneSeparation), -(initialOffset + (i * coneSeparation) + coneColorSeparateion), .25, .25,
                  0.45, cone2);
         }

      }

      private static CombinedTerrainObject3D setUpPath3RampsWithLargeBlocks3D(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         AppearanceDefinition color = YoAppearance.DarkGray();

         // float rampHeight = 0.3f;

         float rampHeight = 0.625f;

         setUpRamp3D(combinedTerrainObject, 5.0f, 0.0f, 2.0f, 3.0f, rampHeight, color);
         setUpWall3D(combinedTerrainObject, new double[] {7.0f, 0.0f}, .5f, 1.0f, rampHeight, 0, color);

         setUpWall3D(combinedTerrainObject, new double[] {7.75f, 0.0f}, 2f, .5f, rampHeight, 0, color);
         setUpWall3D(combinedTerrainObject, new double[] {8.5f, 0f}, .5f, .75f, rampHeight - 0.1, 0, color);

         setUpWall3D(combinedTerrainObject, new double[] {8.5f, .75f}, .5f, .75f, rampHeight, 0, color);

         setUpWall3D(combinedTerrainObject, new double[] {8.5f, -0.66f}, .25f, 1f, rampHeight, 0, color);

         setUpWall3D(combinedTerrainObject, new double[] {8.5f, -1.045f}, .25f, 1f, rampHeight, 0, color);

         setUpWall3D(combinedTerrainObject, new double[] {9.25f, 0f}, 2.0f, 0.5f, rampHeight, 0, color);
         setUpRamp3D(combinedTerrainObject, 11f, 0f, 2.0f, -3.0f, rampHeight, color);

         // Do this for a long ramp for testing:
         // rampHeight = 1.0f;
         // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);

         return combinedTerrainObject;
      }


      private void testRotatableRampsSetupForGraphicsAndCollision(CombinedTerrainObject3D combinedTerrainObject)
      {
         double courseAngleDeg = 45.0;
         double startDistance = 4.0;
         AppearanceDefinition color = YoAppearance.Red();

         final double sectionLength = 2.4384;    // 8 ft
         int numberOfRamps = 2;
         combinedTerrainObject.addTerrainObject(setUpMultipleUpDownRamps("updDownRamp1", courseAngleDeg, startDistance, numberOfRamps, sectionLength, color));

         courseAngleDeg = 90.0;
         startDistance = 4.0;
         color = YoAppearance.Orange();

         combinedTerrainObject.addTerrainObject(setUpMultipleUpDownRamps("updDownRamp2", courseAngleDeg, startDistance, numberOfRamps, sectionLength, color));

         color = YoAppearance.Gray();
         float rampHeight = 0.625f;

         setUpRamp3D(combinedTerrainObject, 5.0f, 0.0f, 2.0f, 3.0f, rampHeight, color);

         // setUpGround();
         // 45,200
         // Ground
//    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
         YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
         double width2 = 10;
         double width1 = width2 / 2;
         RigidBodyTransform location = new RigidBodyTransform();
         location.setTranslation(new Vector3D(width1 / 2, width1 / 2, -0.5));
         RigidBodyTransform location2 = new RigidBodyTransform(location);

         // location2.setTranslation(new Vector3d(0, 0, -2));

         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, width1, width1, 1), texture);
         combinedTerrainObject.addTerrainObject(newBox);
         RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location2, width2, width2, 1), texture);
         combinedTerrainObject.addTerrainObject(newBox2);
      }

      private static void setUpPath4DRCTrialsTrainingWalkingCourseDifficult(CombinedTerrainObject3D combinedTerrainObject)
      {
         double courseAngleDeg = 45.0;
         double startDistance = 4.0;
         AppearanceDefinition color = YoAppearance.Gray();

         final double sectionLength = 2.4384;    // 8 ft

         // need basics:
         // basic ramp
         // basic block (height parameter: # block layers, 1-2 typ)
         // square block (two basic blocks side by side, height parameter: #
         // block layers, 0-4 typ)
         // diagonal block-flat (# of square block base supports, 1 typ, e.g. 1
         // square block under)
         // diagonal block-upright (# of square block base supports, 0 and 1 typ)
         // slanted block (square block on ramp, with # of square block support
         // layers: 0-3 typ)

         // 1. Flat terrain: Pavers and Astroturf. Do nothing, but space out
         // others farther.

         // 2. Ramps (Pitch Ramps 15degrees)
         int numberOfRamps = 2;
         combinedTerrainObject.addTerrainObject(setUpMultipleUpDownRamps("upDownRamp1", courseAngleDeg, startDistance, numberOfRamps, sectionLength, color));

         // 3. Tripping Hazards
         // Diagonal 2x4s and 4x4s
         // From the picture layout:
         // first half has 2x4s(1.5x3.5) flat at 45deg angles spaced about 2ft
         // apart (horiz and vert) (5 total)
         // second half has 4x4s(3.5x3.5) at 45 deg angles spaced 4ft apart (3
         // total)
         // I chose to do a worse case scenario where 2x4 and 4x4 are actually
         // 2x4 and 4x4 (not standard sizes)
         int[] numberOfStepOvers = {5, 3};
         startDistance = setUpTripHazards(combinedTerrainObject, courseAngleDeg, startDistance, numberOfStepOvers, sectionLength, color);

         // 4. Hurdles
         // 15cm (6 in) and 30 cm (12 in)
         // From the picture layout:
         // 1st section: midway (centered at 2ft) blocks layed straight across, 6
         // on bottom layer, 3 on top directly aligned on others. In other
         // layout, 5 on top not directly over bottom.
         // 2nd section: midway (centered at 2ft from start of second section,
         // 45deg zig zag pattern, two high. 8 on bottom, 4 directly on top or 7
         // overlapped.
         startDistance += sectionLength + sectionLength / 4;
         setUpStraightHurdles(combinedTerrainObject, courseAngleDeg, startDistance, new int[] {6, 5});

         startDistance += sectionLength / 2;
         combinedTerrainObject.addTerrainObject(setUpZigZagHurdles("zigZagHurdles", courseAngleDeg, startDistance, new int[] {8, 7}, 45.0));

         startDistance += sectionLength / 4;

         // 5. Footfalls and Holes
         // 80 cm (32 in) and 40 cm (16 in) squares
         // 6. Ascend Flat Top Steps
         // 7. Descend Flat Top Steps
         // 8. Ascend Pitch/Roll 15 deg Top Steps
         // 9. Descend Pitch/Roll 15 deg Top Steps
         setUpCinderBlockField(combinedTerrainObject, courseAngleDeg, startDistance);
         startDistance += sectionLength * 5;

         // 10. Step-Over Obstacles
         setUpStepOverObstacles(combinedTerrainObject, courseAngleDeg, startDistance, color, sectionLength);

      }

      private static CombinedTerrainObject3D setUpPath4DRCTrialsTrainingWalkingCourse(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         double courseAngleDeg = 45.0;
         double startDistance = 4.0;
         AppearanceDefinition color = YoAppearance.Gray();

         final double sectionLength = 2.4384;    // 8 ft

         // 1. Ramp and Zigzag Hurdle (Pitch Ramp 15degrees)
         int numberOfRamps = 1;
         combinedTerrainObject.addTerrainObject(setUpMultipleUpDownRamps("upDownRamp1", courseAngleDeg, startDistance, numberOfRamps, sectionLength / 2, color));

         startDistance += sectionLength;
         combinedTerrainObject.addTerrainObject(setUpZigZagHurdles("zigZagHurdles", courseAngleDeg, startDistance, new int[] {9}, -45.0));

         startDistance += sectionLength / 4;

         // 2a. Ascend Flat Top Steps
         // 2b. Descend Flat Top Steps
         // 3a. Ascend Pitch/Roll 15 deg Top Steps
         // 3b. Descend Pitch/Roll 15 deg Top Steps
         combinedTerrainObject.addTerrainObject(setUpCinderBlockFieldActual("cinderBlockField", courseAngleDeg, startDistance));
         startDistance += sectionLength * 5;

         // 4. Two cinder block high hurdle for testcase purposes only, not part of actual trial obstacle course
         combinedTerrainObject.addTerrainObject(setUpZigZagHurdles("zigZagHurdlesTwoHigh", courseAngleDeg, startDistance, new int[] {8, 7}, 45.0));

         return combinedTerrainObject;
      }

      private CombinedTerrainObject3D setUpTrialsQuals(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         double courseAngleDeg = -45.0;
         double startDistance = 2.0;
         AppearanceDefinition color = YoAppearance.Gray();

         double sectionLength = 1.83;    // 6 ft
         double sectionWidth = 2.43;    // 8 ft
         double borderWidth = 0.1;

         double[] point = {0, 0};
         double[] rotatedPoint;

         // Setup Door
         sectionLength = 1.22;    // 4 ft
         double doorWidth = 0.80;    // 32 inches (rounded down to 90cm according to p37 initial task description dims
         double doorHeight = 2.0;    // 82 inches.
         startDistance += sectionLength;
         point[0] = startDistance;
         double doorCenter = 0;
         double doorJamWidth = 0.05;
         for (int courseSide = -1; courseSide <= 1; courseSide += 2)
         {
            // Door posts
            doorCenter = courseSide * (sectionWidth / 2 - doorWidth / 2 - doorJamWidth);

            for (int doorSide = -1; doorSide <= 1; doorSide += 2)
            {
               point[1] = doorSide * (doorWidth / 2 + borderWidth / 2) + doorCenter;
               point[1] = point[1];
               rotatedPoint = rotateAroundOrigin(point, courseAngleDeg);
               setUpSlopedBox(combinedTerrainObject, rotatedPoint[0], rotatedPoint[1], doorHeight / 2, borderWidth, borderWidth, doorHeight, 0, courseAngleDeg,
                     color);
            }

            ////Door overhead
            // point[1] = doorCenter;
            // rotatedPoint = rotateAroundOrigin(point, courseAngleDeg);
            // setUpSlopedBox(rotatedPoint[0], rotatedPoint[1], doorHeight
            // + borderWidth / 2, borderWidth,
            // doorWidth + 2 * borderWidth, borderWidth, 0,
            // courseAngleDeg, color);
         }

         // Walls
         point[1] = 0;
         rotatedPoint = rotateAroundOrigin(point, courseAngleDeg);
         setUpSlopedBox(combinedTerrainObject, rotatedPoint[0], rotatedPoint[1], (doorHeight + borderWidth) / 2, borderWidth,
               Math.abs(doorCenter) * 2 - doorWidth - 2 * borderWidth, doorHeight + borderWidth, 0, courseAngleDeg, color);

         // setup side walls
         double sideWallHeight = doorHeight / 2.0;
         for (int side = -1; side <= 1; side += 2)
         {
            point[1] = side * (sectionWidth / 2 + borderWidth / 2);
            rotatedPoint = rotateAroundOrigin(point, courseAngleDeg);
            setUpSlopedBox(combinedTerrainObject, rotatedPoint[0], rotatedPoint[1], (sideWallHeight + borderWidth) / 2, sectionLength * 2, borderWidth,
                  sideWallHeight + borderWidth, 0, courseAngleDeg, color);
         }

         startDistance += sectionLength;

         // cinder blocks
         startDistance += sectionLength;
         setUpStraightHurdles(combinedTerrainObject, courseAngleDeg, startDistance, new int[] {6});

         // sides to show boundaries, and virtual start and end lines:
         point[0] = startDistance;

         for (int side = -1; side <= 1; side += 2)
         {
            point[1] = side * (sectionWidth / 2 + borderWidth / 2);
            point[1] = point[1];
            rotatedPoint = rotateAroundOrigin(point, courseAngleDeg);
            setUpSlopedBox(combinedTerrainObject, rotatedPoint[0], rotatedPoint[1], borderWidth / 2, sectionLength * 2, borderWidth, borderWidth, 0,
                  courseAngleDeg, color);
         }

//    startDistance += sectionLength;


         // valve (graphics only)
//    startDistance += sectionLength;

         for (int i = 0; i < 1; i++)
         {
            Graphics3DObject linkGraphics = new Graphics3DObject();

            // Vector3d translation = new Vector3d(-1.0, 0, startDistance);// startDistance);
            Vector3D translation = new Vector3D(-1, 0, 2.9);    // startDistance);

            linkGraphics.rotate(Math.PI / 2, Axis.Y);
            linkGraphics.rotate(Math.toRadians(-courseAngleDeg), Axis.X);
            linkGraphics.translate(translation);

            double outsideRadius = 0.2;
            double gripRadius = 0.022;
            linkGraphics.addArcTorus(0, Math.PI * 2, outsideRadius, gripRadius, YoAppearance.randomColor(random));


            combinedTerrainObject.addStaticLinkGraphics(linkGraphics);    // new
         }

         return combinedTerrainObject;
      }

      private static void setUpStepOverObstacles(CombinedTerrainObject3D combinedTerrainObject, double courseAngleDeg, double startDistance,
                                                 AppearanceDefinition color, final double sectionLength)
      {
         double[] point = {startDistance + sectionLength * 0.75, sectionLength * 0.25};
         double[] newPoint = rotateAroundOrigin(point, courseAngleDeg);
         double cylinderLength = cinderBlockLength * 3.5;
         double cylinderRadius = cinderBlockWidth * 1.5;
         setUpSlopedCylinder(combinedTerrainObject, newPoint[0], newPoint[1], 0.0, cylinderLength, cylinderRadius, 0.0, courseAngleDeg + 45, color);

         point = new double[] {startDistance + sectionLength * 0.75 - 0.3, -sectionLength * 0.25 + .18};
         newPoint = rotateAroundOrigin(point, courseAngleDeg);
         double trussLength = 1.524;
         double trussSide = 0.291;
         setUpTruss(combinedTerrainObject, newPoint, trussLength, trussSide, courseAngleDeg + 45, color);

         point = new double[] {startDistance + sectionLength * 0.75 / 2, sectionLength / 4};
         newPoint = rotateAroundOrigin(point, courseAngleDeg);
         double beamHorizontalLength = sectionLength * 0.75;
         double beamVerticalHeight = cylinderRadius;
         double beamLength = Math.sqrt(beamHorizontalLength * beamHorizontalLength + beamVerticalHeight * beamVerticalHeight);
         double beamSlopeRad = Math.atan(beamVerticalHeight / beamHorizontalLength);
         double beamZLength = 0.09;
         setUpSlopedBox(combinedTerrainObject, newPoint[0], newPoint[1], beamVerticalHeight / 2.0 + beamZLength / 2, beamLength, 0.04, beamZLength, beamSlopeRad,
               courseAngleDeg, color);

         point = new double[] {startDistance + sectionLength * 0.75 / 2, -sectionLength / 6};
         newPoint = rotateAroundOrigin(point, courseAngleDeg);
         beamVerticalHeight = trussSide + 0.13;
         beamLength = Math.sqrt(beamHorizontalLength * beamHorizontalLength + beamVerticalHeight * beamVerticalHeight);
         beamSlopeRad = Math.atan(beamVerticalHeight / beamHorizontalLength);
         setUpSlopedBox(combinedTerrainObject, newPoint[0], newPoint[1], beamVerticalHeight / 2.0 + beamZLength / 2, beamLength, 0.04, beamZLength, beamSlopeRad,
               courseAngleDeg, color);
      }

      private static void setUpTruss(CombinedTerrainObject3D combinedTerrainObject, double[] newPoint, double trussLength, double trussSide,
                                     double courseAngleDeg, AppearanceDefinition color)
      {
         AppearanceDefinition overrideColor = YoAppearance.White();    // color;
         overrideColor.setTransparency(0.95);

         TrussWithSimpleCollisions truss = new TrussWithSimpleCollisions(newPoint, trussLength, trussSide, courseAngleDeg, overrideColor);
         combinedTerrainObject.addTerrainObject(truss);
      }

      private static CombinedTerrainObject3D setUpCinderBlockFieldActual(String name, double courseAngle, double startDistance)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         int nBlocksWide = 6;
         int nBlocksLong = 21;

         double[][] blockAngle = new double[nBlocksLong][nBlocksWide];
         int[][] blockHeight = new int[nBlocksLong][nBlocksWide];
         BLOCKTYPE[][] blockType = new BLOCKTYPE[nBlocksLong][nBlocksWide];
         for (int i = 0; i < nBlocksLong; i++)
         {
            for (int j = 0; j < nBlocksWide; j++)
            {
               blockHeight[i][j] = -1;    // (int) Math.round(Math.random()*4-1);
               blockAngle[i][j] = 0;    // (int) Math.round(Math.random()*3)*45;
               blockType[i][j] = BLOCKTYPE.FLAT;
            }
         }

         blockHeight = new int[][]
               {
                     {
                           -1, -1, -1, -1, -1, 0
                     },
                     {
                           -1, -1, -1, -1, 0, 1
                     },
                     {
                           -1, -1, -1, 0, 1, 2
                     },
                     {
                           -1, -1, 0, 1, 2, 3
                     },
                     {
                           -1, 0, 1, 2, 3, 2
                     },
                     {
                           0, 1, 2, 3, 2, 1
                     },
                     {
                           1, 2, 3, 2, 1, 0
                     },
                     {
                           2, 3, 2, 1, 0, -1
                     },
                     {
                           3, 2, 1, 0, -1, 0
                     },
                     {
                           2, 1, -1, -1, -1, 0
                     },
                     {
                           1, 0, -1, 0, 0, 1
                     },
                     {
                           0, -1, 0, 0, 1, 2
                     },
                     {
                           -1, 0, 0, 1, 2, 3
                     },
                     {
                           0, 0, 1, 2, 3, 2
                     },
                     {
                           0, 1, 2, 3, 2, 1
                     },
                     {
                           1, 2, 3, 2, 1, 0
                     },
                     {
                           2, 3, 2, 1, 0, -1
                     },
                     {
                           3, 2, 1, 0, -1, -1
                     },
                     {
                           2, 1, 0, -1, -1, -1
                     },
                     {
                           1, 0, -1, -1, -1, -1
                     },
                     {
                           0, -1, -1, -1, -1, -1
                     }
               };

         final int NORTH = -90;
         final int SOUTH = 90;
         final int WEST = 0;
         final int EAST = 180;
         final int startAngled = 9;
         for (int i = startAngled; i < nBlocksLong; i++)
         {
            for (int j = Math.max(0, startAngled + (nBlocksWide - 1) - i); j < nBlocksWide; j++)
            {
               boolean evenRow = (i - startAngled) % 2 == 0;
               boolean evenCol = j % 2 == 0;
               blockType[i][j] = BLOCKTYPE.ANGLED;

               if (evenRow)
               {
                  if (evenCol)
                     blockAngle[i][j] = WEST;
                  else
                     blockAngle[i][j] = NORTH;
               }
               else
               {
                  if (evenCol)
                     blockAngle[i][j] = SOUTH;
                  else
                     blockAngle[i][j] = EAST;
               }
            }
         }

         startDistance += cinderBlockLength / 2;

         for (int i = 0; i < nBlocksLong; i++)
         {
            for (int j = 0; j < nBlocksWide; j++)
            {
               double xCenter = startDistance + i * cinderBlockLength;
               double yCenter = (nBlocksWide * cinderBlockLength) / 2 - j * cinderBlockLength - cinderBlockLength / 2;
               double[] point = {xCenter, yCenter};
               double[] rotatedPoint = rotateAroundOrigin(point, courseAngle);
               int h = blockHeight[i][j];
               double deg = blockAngle[i][j] + courseAngle;
               switch (blockType[i][j])
               {
                  case FLAT :
                     setUpCinderBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case FLATSKEW :
                     setUpFlatSkewedBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case UPRIGHTSKEW :
                     setUpSkewedUprightBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case ANGLED :
                     setUpRampBlock(combinedTerrainObject, rotatedPoint, h, deg);

                     break;
               }
            }
         }

         return combinedTerrainObject;
      }

      private static void setUpCinderBlockField(CombinedTerrainObject3D combinedTerrainObject, double courseAngle, double startDistance)
      {
         int nBlocksWide = 6;
         int nBlocksLong = 31;

         double[][] blockAngle = new double[nBlocksLong][nBlocksWide];
         int[][] blockHeight = new int[nBlocksLong][nBlocksWide];
         BLOCKTYPE[][] blockType = new BLOCKTYPE[nBlocksLong][nBlocksWide];
         for (int i = 0; i < nBlocksLong; i++)
         {
            for (int j = 0; j < nBlocksWide; j++)
            {
               blockHeight[i][j] = -1;    // (int) Math.round(Math.random()*4-1);
               blockAngle[i][j] = 0;    // (int) Math.round(Math.random()*3)*45;
               blockType[i][j] = BLOCKTYPE.FLAT;
            }
         }

         blockHeight = new int[][]
               {
                     {
                           0, 0, -1, -1, 0, 0
                     },    // 5. Footfalls and Holes
                     {
                           0, 0, -1, -1, 0, 0
                     },
                     {
                           -1, -1, 0, 0, -1, -1
                     },
                     {
                           -1, 0, 0, 0, 0, -1
                     },
                     {
                           0, -1, 0, -1, 0, 0
                     },
                     {
                           1, 0, -1, 0, -1, 0
                     },
                     {
                           0, 1, 0, -1, 0, -1
                     },    // 6.7. Ascend/Descend Flat Top Steps
                     {
                           2, 0, 1, 0, 1, -1
                     },
                     {
                           3, 2, 0, 1, 0, 1
                     },
                     {
                           2, 3, 2, 0, 1, -1
                     },
                     {
                           1, 2, 3, 2, 0, 1
                     },
                     {
                           1, 1, 2, 3, 2, 0
                     },
                     {
                           1, 1, 1, 2, 3, 2
                     },
                     {
                           0, 0, 1, 1, 2, 3
                     },
                     {
                           0, 0, 0, 1, 1, 2
                     },
                     {
                           0, 0, 0, 0, 1, 1
                     },
                     {
                           0, 0, 0, 0, 0, 1
                     },
                     {
                           0, 0, 0, 0, 0, 0
                     },
                     {
                           0, 0, 0, 0, 0, 0
                     },    // 8.9. Ascend/descend Pitch/Roll 15 deg Top Steps
                     {
                           1, 0, 1, 0, 1, 0
                     },    // 1 angled...
                     {
                           0, 1, 0, 1, 0, 1
                     },
                     {
                           0, 0, 0, 0, 1, 2
                     },
                     {
                           0, 0, 0, 1, 2, 3
                     },
                     {
                           0, 0, 1, 2, 3, 2
                     },
                     {
                           0, 1, 2, 3, 2, 1
                     },
                     {
                           1, 2, 3, 2, 1, 0
                     },
                     {
                           2, 3, 2, 1, 0, 0
                     },
                     {
                           3, 2, 1, 0, 0, 0
                     },
                     {
                           2, 1, 0, 0, 0, 0
                     },
                     {
                           1, 0, 0, 0, 0, 0
                     },
                     {
                           0, 0, 0, 0, 0, 0
                     }
               };

         int[] full90Diags =
               {
                     -1, -3, -5, 1, 7, 8, 9, 13, 15, 17, 19, 21, 23
               };
         int[] alternating0_90DiagsAndUprightSkewed =
               {
                     10, 12, 14, 16, 18, 20, 22
               };

         for (int i = 0; i < full90Diags.length; i++)
         {
            for (int j = Math.max(0, -full90Diags[i]); j < nBlocksWide; j++)
            {
               int col = j;
               int row = full90Diags[i] + col;
               if (row < nBlocksLong)
                  blockAngle[row][col] = 90;
            }
         }

         for (int i = 0; i < alternating0_90DiagsAndUprightSkewed.length; i++)
         {
            for (int j = 0; j < nBlocksWide; j++)
            {
               int col = j;
               int row = alternating0_90DiagsAndUprightSkewed[i] + col;
               blockType[row][col] = BLOCKTYPE.UPRIGHTSKEW;
               if (j % 2 == 1)
                  blockAngle[row][col] = 90;
            }
         }

         final int flatSkewedRow = 19;
         for (int col = 0; col < nBlocksWide - 1; col++)
         {
            boolean evenCol = col % 2 == 0;
            int row = flatSkewedRow + (evenCol ? 0 : 1);
            blockType[row][col] = BLOCKTYPE.FLATSKEW;
            if (evenCol)
               blockAngle[row][col] = 90;
            else
               blockAngle[row][col] = 0;
         }

         final int NORTH = -90;
         final int SOUTH = 90;
         final int WEST = 0;
         final int EAST = 180;
         final int startAngled = 19;
         for (int i = startAngled; i < nBlocksLong; i++)
         {
            for (int j = Math.max(0, startAngled + (nBlocksWide - 1) - i); j < nBlocksWide; j++)
            {
               boolean evenRow = (i - startAngled) % 2 == 0;
               boolean evenCol = j % 2 == 0;
               blockType[i][j] = BLOCKTYPE.ANGLED;

               if (evenRow)
               {
                  if (evenCol)
                     blockAngle[i][j] = WEST;
                  else
                     blockAngle[i][j] = NORTH;
               }
               else
               {
                  if (evenCol)
                     blockAngle[i][j] = SOUTH;
                  else
                     blockAngle[i][j] = EAST;
               }
            }
         }

         startDistance += cinderBlockLength / 2;

         for (int i = 0; i < nBlocksLong; i++)
         {
            for (int j = 0; j < nBlocksWide; j++)
            {
               double xCenter = startDistance + i * cinderBlockLength;
               double yCenter = (nBlocksWide * cinderBlockLength) / 2 - j * cinderBlockLength - cinderBlockLength / 2;
               double[] point = {xCenter, yCenter};
               double[] rotatedPoint = rotateAroundOrigin(point, courseAngle);
               int h = blockHeight[i][j];
               double deg = blockAngle[i][j] + courseAngle;
               switch (blockType[i][j])
               {
                  case FLAT :
                     setUpCinderBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case FLATSKEW :
                     setUpFlatSkewedBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case UPRIGHTSKEW :
                     setUpSkewedUprightBlockSquare(combinedTerrainObject, rotatedPoint, h, deg);

                     break;

                  case ANGLED :
                     setUpRampBlock(combinedTerrainObject, rotatedPoint, h, deg);

                     break;
               }
            }
         }
      }

      private static CombinedTerrainObject3D setUpMultipleUpDownRamps(String name, double courseAngleDegrees, double startDistance, int numberOfRamps,
                                                                      final double sectionLength, AppearanceDefinition color)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         for (int i = 1; i <= 2 * numberOfRamps; i = i + 2)
         {
            double rampLength = sectionLength / (numberOfRamps * 2);
            double rampAngle = Math.toRadians(15);
            double rampHeight = rampLength * Math.tan(rampAngle);
            double rampCenter = startDistance + rampLength * (i - 1) + rampLength / 2;
            double[] newPoint = rotateAroundOrigin(new double[] {rampCenter, 0}, courseAngleDegrees);
            setUpRotatedRamp(combinedTerrainObject, newPoint[0], newPoint[1], sectionLength, rampLength, rampHeight, courseAngleDegrees, color);

            double rampDownCenter = startDistance + rampLength * (i) + rampLength / 2;
            newPoint = rotateAroundOrigin(new double[] {rampDownCenter, 0}, courseAngleDegrees);
            setUpRotatedRamp(combinedTerrainObject, newPoint[0], newPoint[1], sectionLength, -rampLength, rampHeight, courseAngleDegrees, color);
         }

         return combinedTerrainObject;
      }

      private static double setUpTripHazards(CombinedTerrainObject3D combinedTerrainObject, double courseAngle, double startDistance, int[] numberOfStepOvers,
                                             final double sectionLength, AppearanceDefinition color)
      {
         double[] stepHeight = {0.0508, 0.1016};
         double[] stepWidth = {0.1016, 0.1016};
         double[] degreesOffset = {45, -45};

         startDistance += sectionLength;

         for (int i = 0; i < numberOfStepOvers.length; i++)
         {
            for (int j = 0; j < numberOfStepOvers[i]; j++)
            {
               double stepLength;
               if (Math.abs(degreesOffset[i]) < Math.toDegrees(Math.atan(sectionLength / (sectionLength / 2))))
                  stepLength = (sectionLength / 2) / Math.cos(Math.toRadians(degreesOffset[i]));
               else
                  stepLength = (sectionLength) / Math.sin(Math.toRadians(degreesOffset[i]));
               double[] point = {startDistance + sectionLength / 4 + sectionLength / 2 * i, -sectionLength / 2 + j * sectionLength / (numberOfStepOvers[i] - 1)};
               double[] newPoint = rotateAroundOrigin(point, courseAngle);
               setUpWall3D(combinedTerrainObject, newPoint, stepWidth[i], stepLength, stepHeight[i], courseAngle + degreesOffset[i], color);
            }
         }

         return startDistance;
      }

      private static void setUpStraightHurdles(CombinedTerrainObject3D combinedTerrainObject, double courseAngle, double startDistance,
                                               int[] numberStraightHurdles)
      {
         for (int i = 0; i < numberStraightHurdles.length; i++)
         {
            for (int j = 0; j < numberStraightHurdles[i]; j++)
            {
               double[] point = {startDistance, -(numberStraightHurdles[i] * cinderBlockLength) / 2 + j * cinderBlockLength + cinderBlockLength / 2};
               double[] newPoint = rotateAroundOrigin(point, courseAngle);
               setUpCinderBlock(combinedTerrainObject, newPoint, i, courseAngle + 90);
            }
         }
      }

      private static CombinedTerrainObject3D setUpZigZagHurdles(String name, double courseAngle, double startDistance, int[] numberZigZagHurdles,
                                                                double orientation)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         double xOffset = cinderBlockLength / 4 * Math.cos(Math.toRadians(45));
         double yOffset = cinderBlockLength * Math.cos(Math.toRadians(45));

         for (int i = 0; i < numberZigZagHurdles.length; i++)
         {
            int start45sign = (Math.round(numberZigZagHurdles[i] / 2.0 + .25) % 2 == 0) ? 1 : -1;    // start45 when n=3,4,7,8,11,12,...
            int startXsign = (Math.round((numberZigZagHurdles[i] + 1.) / 2.0 + .25) % 2 == 0) ? -1 : 1;    // start x+ when n=1, 4,5, 8,9, ...
            for (int j = 0; j < numberZigZagHurdles[i]; j++)
            {
               int evenBlockSign = (j % 2 == 0) ? 1 : -1;
               double signedXOffset = xOffset * evenBlockSign * startXsign;
               double signedAngleOffset = orientation * evenBlockSign * start45sign;
               double[] point = {startDistance + signedXOffset, ((numberZigZagHurdles[i] - 1) * yOffset) / 2 - j * yOffset};
               double[] newPoint = rotateAroundOrigin(point, courseAngle);
               setUpCinderBlock(combinedTerrainObject, newPoint, i, courseAngle + signedAngleOffset);
            }
         }

         return combinedTerrainObject;
      }

      private static void setUpRampBlock(CombinedTerrainObject3D combinedTerrainObject, double[] point, int h, double deg)
      {
         setUpRampBlock(combinedTerrainObject, point[0], point[1], h, deg);
      }

      private static void setUpSkewedUprightBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double[] point, int h, double deg)
      {
         setUpSkewedUprightBlockSquare(combinedTerrainObject, point[0], point[1], h, deg);
      }

      private static void setUpFlatSkewedBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double[] point, int h, double deg)
      {
         setUpFlatSkewedBlockSquare(combinedTerrainObject, point[0], point[1], h, deg);
      }

      private static void setUpCinderBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double[] point, int h, double deg)
      {
         setUpCinderBlockSquare(combinedTerrainObject, point[0], point[1], h, deg);
      }

      private CombinedTerrainObject3D setUpPath5NarrowDoor(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         AppearanceDefinition color = YoAppearance.DarkGray();

         // angled Door
         // door1
         setUpWall(combinedTerrainObject, new double[] {0.769f, -9.293f}, 0.973f, 0.157f, 2.5f, -115.0f, color);

         // door2
         setUpWall(combinedTerrainObject, new double[] {-.642f, -8.635f}, 0.973f, 0.157f, 2.54f, -115.0f, color);

         // box2
         setUpWall(combinedTerrainObject, new double[] {-0.485f, -6.573f}, 0.5f, 0.5f, 1.0f, -45, color);

         // box1
         setUpWall(combinedTerrainObject, new double[] {0.515f, -4.972f}, 0.5f, 0.5f, 1.0f, -110.0f, color);

         return combinedTerrainObject;
      }

      private CombinedTerrainObject3D setUpPath6Barriers(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         AppearanceDefinition color = YoAppearance.DarkGray();
         double courseAngle = -135.0;
         int numberOfStepOvers = 8;
         double heightIncrease = 0.05;
         double startDistance = 4.0;
         double spacing = 1.0;

         double barrierWidth = 3.0;
         double platformWidth = 0.8;

         for (int i = 0; i < numberOfStepOvers; i++)
         {
            double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * spacing), 0}, courseAngle);
            setUpWall(combinedTerrainObject, newPoint, barrierWidth, 0.15, heightIncrease * (i + 1), courseAngle, color);
         }

         for (int i = 0; i < numberOfStepOvers; i++)
         {
            double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * spacing), (barrierWidth - platformWidth) / 2.0 + 0.001}, courseAngle);
            setUpWall(combinedTerrainObject, newPoint, platformWidth, 0.4 * spacing, heightIncrease * (i + 1) + 0.001, courseAngle, color);
         }

         return combinedTerrainObject;
      }

      private CombinedTerrainObject3D setUpPath7Stairs(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         AppearanceDefinition color = YoAppearance.DarkGray();
         double courseAngle = 135;
         int numberOfSteps = 3;
         double rise = 0.2;
         double startDistance = 4.0;
         double run = 0.4;

         for (int i = 0; i < numberOfSteps; i++)
         {
            double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * run), 0}, courseAngle);
            setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (i + 1), courseAngle, color);
         }

         {
            double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (numberOfSteps * run), 0}, courseAngle);
            setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (numberOfSteps - 1 + 1), courseAngle, color);
         }

         for (int i = 1; i < numberOfSteps + 1; i++)
         {
            double offset = numberOfSteps * run;
            double[] newPoint = rotateAroundOrigin(new double[] {offset + startDistance + (i * run), 0}, courseAngle);
            setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (-i + numberOfSteps + 1), courseAngle, color);
         }

         return combinedTerrainObject;
      }

      private CombinedTerrainObject3D setUpPath8RampsWithSteppingStones(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

         AppearanceDefinition color = YoAppearance.DarkGray();

         float rampHeight = 0.3f;

         // ramp up and landing
         setUpRamp3D(combinedTerrainObject, -5.0f, 0.0f, 3.0f, -3.0f, rampHeight, color);
         setUpWall(combinedTerrainObject, new double[] {-7.0f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

         // simple stepping stones, centered at x=-0.75m
         setUpWall(combinedTerrainObject, new double[] {-7.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-8.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-9.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);

         // qualification stepping stones, centered along x=0.75m
         setUpWall(combinedTerrainObject, new double[] {-8.0f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-8.5f, 0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-9.3f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);

         // middle landing
         setUpWall(combinedTerrainObject, new double[] {-10.5f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

         if (DIFFICULT_STEPPING_STONES)
         {
            // more difficult stepping stones
            setUpWall(combinedTerrainObject, new double[] {-11.6f, -0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
            setUpWall(combinedTerrainObject, new double[] {-12.2f, 0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
            setUpWall(combinedTerrainObject, new double[] {-13.1f, 0.15f}, 0.5f, 0.5f, rampHeight, 0, color);
            setUpWall(combinedTerrainObject, new double[] {-14f, 0.95f}, 0.5f, 0.5f, rampHeight, 0, color);

            // landing and ramp down
            setUpWall(combinedTerrainObject, new double[] {-15.5f, 0.5f}, 2.0f, 1.0f, rampHeight, 0, color);
            setUpRamp3D(combinedTerrainObject, -17.5f, 0.5f, 2.0f, 3.0f, rampHeight, color);
         }
         else
         {
            setUpRamp3D(combinedTerrainObject, -12.5f, 0.0f, 3.0f, 3.0f, rampHeight, color);
         }

         // Do this for a long ramp for testing:
         // rampHeight = 1.0f;
         // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);

         return combinedTerrainObject;
      }

      private static void addLimboBar(CombinedTerrainObject3D combinedTerrainObject)
      {
         double height = 1;
         double width = 1.5;
         AppearanceDefinition color = YoAppearance.DarkGray();

         setUpWall(combinedTerrainObject, new double[] {1, width / 2}, 0.125, 0.125, height, 0, color);
         setUpWall(combinedTerrainObject, new double[] {1, -width / 2}, 0.125, 0.125, height, 0, color);

         combinedTerrainObject.getLinkGraphics().translate(0, width / 2, height);
         combinedTerrainObject.getLinkGraphics().addCube(0.125, width, 0.125, color);
         combinedTerrainObject.getLinkGraphics().translate(0, -width / 2, -height);

      }

      private CombinedTerrainObject3D addRocks3D(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(name);

         for (int i = 0; i < NUM_ROCKS; i++)
         {
            double centroidHeight = random.nextDouble() * MAX_ROCK_CENTROID_HEIGHT;
            Vector3D normal = generateRandomUpFacingNormal();

            double[] approximateCentroid = generateRandomApproximateCentroid(i);

            double[][] vertices = generateRandomRockVertices(approximateCentroid[0], approximateCentroid[1]);

            addRock3D(combinedTerrainObject3D, normal, centroidHeight, vertices);
         }

         Graphics3DObject linkGraphics = new Graphics3DObject();

//    linkGraphics.rotate(Math.PI / 2, Axis.Y);
//    linkGraphics.rotate(Math.toRadians(-courseAngleDeg), Axis.X);
         linkGraphics.translate(new Vector3D(2, 2, 0));


         if (SHOW_FULL_TESTBED)
            linkGraphics.addModelFile("models/ManualTestBed.obj");
         else
            linkGraphics.addModelFile("models/QAPlanGrid.obj");


         combinedTerrainObject3D.addStaticLinkGraphics(linkGraphics);

         return combinedTerrainObject3D;
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

      private double[] generateRandomApproximateCentroid(int position)
      {
         double[] approximateCentroid = new double[2];

         if (FULLY_RANDOM)
         {
            approximateCentroid[0] = random.nextDouble() * ROCK_FIELD_WIDTH - ROCK_FIELD_WIDTH / 2.0;
            approximateCentroid[1] = random.nextDouble() * ROCK_PATH_LENGTH + ROCKS_START_Y;

         }
         else
         {
            int row = position / ROCKS_PER_ROW;
            int rows = NUM_ROCKS / ROCKS_PER_ROW;
            double distancePerRow = ROCK_PATH_LENGTH / ((double) rows - 1);
            approximateCentroid[1] = ROCKS_START_Y + distancePerRow * row;

            int positionOnRow = position - row * ROCKS_PER_ROW;
            approximateCentroid[0] = ROCK_FIELD_WIDTH * positionOnRow / ROCKS_PER_ROW - ROCK_FIELD_WIDTH / 2.0;
         }

         return approximateCentroid;
      }

      private Vector3D generateRandomUpFacingNormal()
      {
         double normalX = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
         double normalY = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
         Vector3D normal = new Vector3D(normalX, normalY, 1.0);

         return normal;
      }

      private double[][] generateRandomRockVertices(double approximateCentroidX, double approximateCentroidY)
      {
         double[][] vertices = new double[POINTS_PER_ROCK][2];

         for (int j = 0; j < POINTS_PER_ROCK; j++)
         {
            vertices[j][0] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidX - ROCK_BOUNDING_BOX_WIDTH / 2.0;
            vertices[j][1] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidY - ROCK_BOUNDING_BOX_WIDTH / 2.0;
         }

         return vertices;
      }

      private static void addRock3D(CombinedTerrainObject3D combinedTerrainObject, Vector3D normal, double centroidHeight, double[][] vertices)
      {
         AppearanceDefinition color = YoAppearance.DarkGray();

         ArrayList<Point2D> vertexPoints = new ArrayList<Point2D>();

         for (double[] point : vertices)
         {
            Point2D point2d = new Point2D(point);
            vertexPoints.add(point2d);
         }

         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
         RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, color);
         combinedTerrainObject.addTerrainObject(rock);
      }

      private static void addRock(CombinedTerrainObject3D combinedTerrainObject, Vector3D normal, double centroidHeight, double[][] vertices)
      {
         AppearanceDefinition color = YoAppearance.DarkGray();

         ArrayList<Point2D> vertexPoints = new ArrayList<Point2D>();

         for (double[] point : vertices)
         {
            Point2D point2d = new Point2D(point);
            vertexPoints.add(point2d);
         }

         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
         RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, color);
         combinedTerrainObject.addTerrainObject(rock);
      }

      private static void setUpWall3D(CombinedTerrainObject3D combinedTerrainObject, double[] xy, double width, double length, double height, double yawDegrees,
                                      AppearanceDefinition app)
      {
         double x = xy[0];
         double y = xy[1];
         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         location.setTranslation(new Vector3D(x, y, height / 2));
         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpWall(CombinedTerrainObject3D combinedTerrainObject, double[] xy, double width, double length, double height, double yawDegrees,
                                    AppearanceDefinition app)
      {
         double x = xy[0];
         double y = xy[1];
         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         location.setTranslation(new Vector3D(x, y, height / 2));
         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpFloatingStair(CombinedTerrainObject3D combinedTerrainObject, double[] centerPoint, double width, double tread, double thickness,
                                             double stairTopHeight, double yawDegrees, AppearanceDefinition app)
      {
         double xCenter = centerPoint[0];
         double yCenter = centerPoint[1];
         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         location.setTranslation(new Vector3D(xCenter, yCenter, stairTopHeight - thickness / 2));
         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, tread, width, thickness), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpCone(CombinedTerrainObject3D combinedTerrainObject, double x, double y, double bottomWidth, double topWidth, double height,
                                    AppearanceDefinition app)
      {
         combinedTerrainObject.addCone(x, y, bottomWidth, topWidth, height, app);
      }

      private static void setUpRamp(CombinedTerrainObject3D combinedTerrainObject, double x, double y, double width, double length, double height,
                                    AppearanceDefinition app)
      {
         combinedTerrainObject.addRamp(x - length / 2.0, y - width / 2.0, x + length / 2.0, y + width / 2.0, height, app);
      }

      private static void setUpRamp3D(CombinedTerrainObject3D combinedTerrainObject, double x, double y, double width, double length, double height,
                                      AppearanceDefinition app)
      {
         combinedTerrainObject.addRamp(x - length / 2.0, y - width / 2.0, x + length / 2.0, y + width / 2.0, height, app);
      }

      private static void setUpRotatedRamp(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, double width, double run, double rise,
                                           double yawDegreesAboutCenter, AppearanceDefinition app)
      {
         combinedTerrainObject.addRotatedRamp(xCenter, yCenter, run, width, rise, yawDegreesAboutCenter, app);
      }

      // need basics:
      // basic ramp
      // basic block (height parameter: # block layers, 1-2 typ)
      // square block (two basic blocks side by side, height parameter: # block
      // layers, 0-4 typ)
      // diagonal block-flat (# of square block base supports, 1 typ, e.g. 1
      // square block under)
      // diagonal block-upright (# of square block base supports, 0 and 1 typ)
      // slanted block (square block on ramp, with # of square block support
      // layers: 0-3 typ)

      private static void setUpCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                           double yawDegrees)
      {
         double[] centerPoint = {xCenter, yCenter};
         setUpCinderBlock(combinedTerrainObject, centerPoint, numberFlatSupports, yawDegrees);
      }

      private static void setUpCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double[] centerPoint, int numberFlatSupports, double yawDegrees)
      {
         if (numberFlatSupports < 0)
            return;

         AppearanceDefinition app = cinderBlockAppearance;

         double xCenter = centerPoint[0];
         double yCenter = centerPoint[1];

         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         location.setTranslation(new Vector3D(xCenter, yCenter, cinderBlockHeight / 2 + numberFlatSupports * cinderBlockHeight));
         RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3d(location, cinderBlockLength + overlapToPreventGaps, cinderBlockWidth + overlapToPreventGaps,
               cinderBlockHeight + overlapToPreventGaps), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpSlopedBox(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, double zCenter, double xLength,
                                         double yLength, double zLength, double slopeRadians, double yawDegrees, AppearanceDefinition app)
      {
         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         RigidBodyTransform tilt = new RigidBodyTransform();
         tilt.setRotationPitchAndZeroTranslation(-slopeRadians);
         location.multiply(tilt);

         location.setTranslation(new Vector3D(xCenter, yCenter, zCenter));
         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, xLength, yLength, zLength), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpSlopedCylinder(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, double zCenter, double xLength,
                                              double radius, double slopeRadians, double yawDegrees, AppearanceDefinition app)
      {
         double pitchDownDegrees = Math.toDegrees(-slopeRadians + Math.PI / 2);
         Vector3D center = new Vector3D(xCenter, yCenter, zCenter);

         CylinderTerrainObject newCylinder = new CylinderTerrainObject(center, pitchDownDegrees, yawDegrees, xLength, radius, app);
         combinedTerrainObject.addTerrainObject(newCylinder);
      }

      private static void setUpSlopedCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                                 double yawDegrees)
      {
         if (numberFlatSupports < 0)
            return;

         AppearanceDefinition app = cinderBlockAppearance;

         RigidBodyTransform location = new RigidBodyTransform();
         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

         RigidBodyTransform tilt = new RigidBodyTransform();
         tilt.setRotationPitchAndZeroTranslation(-cinderBlockTiltRadians);
         location.multiply(tilt);

         double zCenter = (cinderBlockHeight * Math.cos(cinderBlockTiltRadians) + cinderBlockLength * Math.sin(cinderBlockTiltRadians)) / 2;
         location.setTranslation(new Vector3D(xCenter, yCenter, zCenter + numberFlatSupports * cinderBlockHeight));
         RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth,
               cinderBlockHeight), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpCinderBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                                 double yawDegrees)
      {
         double xOffset = 0, yOffset = cinderBlockWidth / 2.0;
         double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
         double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);

         setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
         setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);

         if (numberFlatSupports > 0)
            setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees + 90);
      }

      private static void setUpFlatSkewedBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                                     double yawDegrees)
      {
         setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees);
         setUpCinderBlock(combinedTerrainObject, xCenter, yCenter, numberFlatSupports, yawDegrees - 45);
      }

      private static void setUpCinderBlockUpright(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                                  double yawDegrees)
      {
         if (numberFlatSupports < 0)
            return;

         AppearanceDefinition app = cinderBlockAppearance;

         // wall
         RigidBodyTransform location = new RigidBodyTransform();
         RigidBodyTransform setUpright = new RigidBodyTransform();

         location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));
         setUpright.setRotationRollAndZeroTranslation(Math.toRadians(90));
         location.multiply(setUpright);

         location.setTranslation(new Vector3D(xCenter, yCenter, cinderBlockWidth / 2 + numberFlatSupports * cinderBlockHeight));
         RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth,
               cinderBlockHeight), app);
         combinedTerrainObject.addTerrainObject(newBox);
      }

      private static void setUpSkewedUprightBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports,
                                                        double yawDegrees)
      {
         setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees);
         setUpCinderBlockUpright(combinedTerrainObject, xCenter, yCenter, numberFlatSupports, yawDegrees - 45);
      }

      private static void setUpRampBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
      {
         if (numberFlatSupports < 0)
            return;

         setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees);

         double rampRise = cinderBlockLength * Math.sin(cinderBlockTiltRadians);

         RigidBodyTransform blockSupportLocation = new RigidBodyTransform();
         blockSupportLocation.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));
         double[] xySupportRotatedOffset = rotateAroundOrigin(new double[] {(cinderBlockLength - rampRise) / 2, 0}, yawDegrees);
         blockSupportLocation.setTranslation(new Vector3D(xCenter + xySupportRotatedOffset[0], yCenter + xySupportRotatedOffset[1],
               rampRise / 2 + numberFlatSupports * cinderBlockHeight));
         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(blockSupportLocation, rampRise, cinderBlockLength, rampRise),
               cinderBlockAppearance);
         combinedTerrainObject.addTerrainObject(newBox);

         double xOffset = 0, yOffset = cinderBlockWidth / 2;
         double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
         double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);
         setUpSlopedCinderBlock(combinedTerrainObject, xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
         setUpSlopedCinderBlock(combinedTerrainObject, xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);
      }

      private static CombinedTerrainObject3D setUpGround(String name)
      {
         CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

//    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
         YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

         RigidBodyTransform location = new RigidBodyTransform();
         location.setTranslation(new Vector3D(0, 0, -0.5));

         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), texture);
         combinedTerrainObject.addTerrainObject(newBox);
         RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 200, 200, 0.75), YoAppearance.DarkGray());
         combinedTerrainObject.addTerrainObject(newBox2);

         return combinedTerrainObject;
      }

      public TerrainObject3D getTerrainObject3D()
      {
         return combinedTerrainObject3D;
      }
}
