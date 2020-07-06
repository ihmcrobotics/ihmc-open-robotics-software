package us.ihmc.avatar.environments;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationConstructionSetTools.util.environments.AdjustableStairsEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.Fiducial;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class PhaseOneDemoEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double WALL_WIDTH = ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX();
   private static final double WALL_DEPTH = 0.05;
   private static final double WALL_HEIGHT = 2.4384;

   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   private final Point3D pullDoorLocation = new Point3D(0, -3.5, 0);
   private final Point3D pushDoorLocation = new Point3D(0, -3.5, 0);

   private final List<PlanarRegionsList> planarRegionsLists = new ArrayList<>();
   private final List<AppearanceDefinition> appearances = new ArrayList<>();

   private final Point3D doorLocation = new Point3D(7.5, -0.5, 0.0);
   private final Point3D stairsLocation = new Point3D(12.0, 0.0, 0.0);
   private final double doorYaw = 0.5 * Math.PI;

   public PhaseOneDemoEnvironment(boolean door, boolean debris, boolean barrel, boolean stairs, boolean cinderBlockField)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      if (door)
      {
        // createPushDoor(pushDoorLocation);
         createPullDoor(pullDoorLocation);
      }
      if (barrel)
         createBarrel();
      if (debris)
         createDebris();
      if (stairs)
         createStairs();
      if (cinderBlockField)
         createCinderBlockField();

      addGroundRegion();

      PlanarRegionsList[] planarRegionsList = this.planarRegionsLists.toArray(new PlanarRegionsList[0]);
      AppearanceDefinition[] appearances = this.appearances.toArray(new AppearanceDefinition[0]);

      PlanarRegionEnvironmentTools.addRegionsToEnvironment(combinedTerrainObject, planarRegionsList, appearances, 0.02);
   }

   private void addGroundRegion()
   {
      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(-3.0, 3.0);
      groundPolygon.addVertex(-3.0, -3.0);
      groundPolygon.addVertex(20.0, -3.0);
      groundPolygon.addVertex(20.0, 3.0);
      groundPolygon.update();
      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      addRegions(new PlanarRegionsList(groundRegion), YoAppearance.LightGray());
   }

   private void createDebris()
   {
      throw new NotImplementedException("Debris not implemented");
   }

   private void createPullDoor(Point3D location)
   {
      ContactableDoorRobot door = new ContactableDoorRobot("PullDoorRobot", location, doorYaw, Fiducial.FIDUCIAL150);
      contactableRobots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);


      combinedTerrainObject.addBox(location.getX()
            - 1.2192, location.getY() - 0.025, location.getX() + 0, location.getY() + 0.025, WALL_HEIGHT, YoAppearance.Bisque());
      combinedTerrainObject.addBox(location.getX() + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(),
                                   location.getY() - 0.025,
                                   location.getX() + 1.2192 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(),
                                   location.getY() + 0.025,
                                   WALL_HEIGHT,
                                   YoAppearance.Beige());

   }
   
   
   private void createPushDoor(Point3D location)
   {
      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", doorLocation, doorYaw, Fiducial.FIDUCIAL50);
      contactableRobots.add(door);
      door.getPinJoint().setQ(Math.toRadians(180));
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);

      combinedTerrainObject.addBox(location.getX()
            + 1.2192, location.getY() - 0.025, location.getX() + 0, location.getY() + 0.025, WALL_HEIGHT, YoAppearance.Bisque());
      combinedTerrainObject.addBox(location.getX() - ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(),
                                   location.getY() - 0.025,
                                   location.getX() - 1.2192 - ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(),
                                   location.getY() + 0.025,
                                   WALL_HEIGHT,
                                   YoAppearance.Beige());

      RigidBodyTransform wall1Transform = new RigidBodyTransform();
      wall1Transform.getTranslation().set(doorLocation);
      wall1Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall1Transform.appendTranslation(- 0.5 * WALL_WIDTH, - 0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      RigidBodyTransform wall2Transform = new RigidBodyTransform();
      wall2Transform.getTranslation().set(doorLocation);
      wall2Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall2Transform.appendTranslation(ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0, 0.0);
      wall2Transform.appendTranslation(0.5 * WALL_WIDTH, 0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      combinedTerrainObject.addRotatableBox(wall1Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
      combinedTerrainObject.addRotatableBox(wall2Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
   }

   private void createCinderBlockField()
   {
      RigidBodyTransform startingBlockTransform = new RigidBodyTransform();
      startingBlockTransform.getTranslation().set(-2.0, -1.0, 0.0);
      startingBlockTransform.getRotation().setYawPitchRoll(Math.toRadians(20.0), 0.0, 0.0);

      addRegions(BehaviorPlanarRegionEnvironments.generateStartingBlockRegions(startingBlockTransform), YoAppearance.Grey());
      addRegions(BehaviorPlanarRegionEnvironments.createRoughUpAndDownStairsWithFlatTop(false), YoAppearance.Grey());
   }

   private void addRegions(PlanarRegionsList regionsToAdd, AppearanceDefinition appearance)
   {
      planarRegionsLists.add(regionsToAdd);
      appearances.add(appearance);
   }

   public PlanarRegionsList getAllRegions()
   {
      PlanarRegionsList combinedRegions = new PlanarRegionsList();
      for (int i = 0; i < planarRegionsLists.size(); i++)
      {
         PlanarRegionsList planarRegionsList = planarRegionsLists.get(i);
         for (int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(j);
            planarRegion.setRegionId(combinedRegions.getNumberOfPlanarRegions());
            combinedRegions.addPlanarRegion(planarRegion);
         }
      }

      return combinedRegions;
   }

   private void createBarrel()
   {
      throw new NotImplementedException("Barrel not implemented");
   }

   private void createStairs()
   {
      double inchesPerMeter = 39.3701;
      int numberOfSteps = 5;
      double stepDepth = 11.0 / inchesPerMeter;
      double stepHeight = 6.75 / inchesPerMeter;
      double stepWidth = 1.0;
      double topPlatformWidth = 1.0;
      double topPlatformLength = 1.0;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(stairsLocation);

      // stairs
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         generator.addCubeReferencedAtBottomMiddle(stepDepth, stepWidth, stepHeight * (i + 1));
         generator.translate(stepDepth, 0.0, 0.0);
      }

      generator.addCubeReferencedAtBottomMiddle(topPlatformLength, topPlatformWidth, stepHeight * numberOfSteps);
      addRegions(generator.getPlanarRegionsList(), YoAppearance.Grey());
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return contactableRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);

      for (Robot r : contactableRobots)
      {
         if (r instanceof Contactable)
            contactController.addContactable((Contactable) r);

      }
      if (contactableRobots.size() > 0)
         contactableRobots.get(0).setController(contactController);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }

   private static final double cinderBlockLength = 0.40; // 40 cm (approx 16 in, just less than 16in)
   private static final double cinderBlockWidth = cinderBlockLength / 2.0;
   private static final double cinderBlockHeight = 0.15; // 15 cm (approx 6 in, less than 6 in, but consistent with other cm measurements)
   private static final double overlapToPreventGaps = 0.002;
   private static final AppearanceDefinition cinderBlockAppearance = YoAppearance.DarkGray();
   private static final double cinderBlockTiltDegrees = 15;
   private static final double cinderBlockTiltRadians = Math.toRadians(cinderBlockTiltDegrees);

   private CombinedTerrainObject3D setUpCinderBlockFieldActual(String name, double courseAngle, double startDistance, double leftRightOffset)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      int nBlocksWide = 6;
      int nBlocksLong = 7;

      double[][] blockAngle = new double[nBlocksLong][nBlocksWide];
      int[][] blockHeight = new int[nBlocksLong][nBlocksWide];
      DefaultCommonAvatarEnvironment.BLOCKTYPE[][] blockType = new DefaultCommonAvatarEnvironment.BLOCKTYPE[nBlocksLong][nBlocksWide];
      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            blockHeight[i][j] = -1; // (int) Math.round(Math.random()*4-1);
            blockAngle[i][j] = 0; // (int) Math.round(Math.random()*3)*45;
            blockType[i][j] = DefaultCommonAvatarEnvironment.BLOCKTYPE.ANGLED;
         }
      }

      blockHeight = new int[][] {{0, 0, 0, 0, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 1, 1, 1, 1, 0}, {1, 2, 1, 1, 2, 1}, {1, 1, 1, 1, 1, 1},
            {0, 0, 0, 0, 0, 0}};

      final int NORTH = 0;
      final int SOUTH = 180;
      final int WEST = 90;
      final int EAST = -90;

      blockAngle = new double[][] {{NORTH, EAST, SOUTH, WEST, NORTH, EAST}, {WEST, NORTH, EAST, SOUTH, WEST, NORTH}, {SOUTH, WEST, NORTH, EAST, SOUTH, WEST},
            {EAST, SOUTH, WEST, NORTH, EAST, SOUTH}, {NORTH, EAST, SOUTH, WEST, NORTH, EAST}, {WEST, NORTH, EAST, SOUTH, WEST, NORTH},
            {SOUTH, WEST, NORTH, EAST, SOUTH, WEST}};

      startDistance += cinderBlockLength / 2;

      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            double xCenter = startDistance + i * cinderBlockLength;
            double yCenter = leftRightOffset + (nBlocksWide * cinderBlockLength) / 2 - j * cinderBlockLength - cinderBlockLength / 2;
            double[] point = {xCenter, yCenter};
            double[] rotatedPoint = rotateAroundOrigin(point, courseAngle);
            int h = blockHeight[i][j];
            double deg = blockAngle[i][j] + courseAngle;
            setUpRampBlock(combinedTerrainObject, rotatedPoint, h, deg);

         }
      }

      return combinedTerrainObject;
   }

   private void setUpRampBlock(CombinedTerrainObject3D combinedTerrainObject, double[] point, int h, double deg)
   {
      setUpRampBlock(combinedTerrainObject, point[0], point[1], h, deg);
   }

   private void setUpRampBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees);

      double rampRise = cinderBlockLength * Math.sin(cinderBlockTiltRadians);

      RigidBodyTransform blockSupportLocation = new RigidBodyTransform();
      blockSupportLocation.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));
      double[] xySupportRotatedOffset = rotateAroundOrigin(new double[] {(cinderBlockLength - rampRise) / 2, 0}, yawDegrees);
      blockSupportLocation.getTranslation()
                          .set(new Vector3D(xCenter + xySupportRotatedOffset[0],
                                            yCenter + xySupportRotatedOffset[1],
                                            rampRise / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(blockSupportLocation, rampRise, cinderBlockLength, rampRise),
                                                                       cinderBlockAppearance);
      combinedTerrainObject.addTerrainObject(newBox);

      double xOffset = 0, yOffset = cinderBlockWidth / 2;
      double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);
      setUpSlopedCinderBlock(combinedTerrainObject, xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
      setUpSlopedCinderBlock(combinedTerrainObject, xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);
   }

   private void setUpSlopedCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
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
      location.getTranslation().set(new Vector3D(xCenter, yCenter, zCenter + numberFlatSupports * cinderBlockHeight));
      RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3D(location,
                                                                                                 cinderBlockLength,
                                                                                                 cinderBlockWidth,
                                                                                                 cinderBlockHeight),
                                                                                       app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpCinderBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double xOffset = 0, yOffset = cinderBlockWidth / 2.0;
      double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);

      setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
      setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);

      if (numberFlatSupports > 0)
         setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees + 90);
   }

   private void setUpCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double[] centerPoint = {xCenter, yCenter};
      setUpCinderBlock(combinedTerrainObject, centerPoint, numberFlatSupports, yawDegrees);
   }

   private double[] rotateAroundOrigin(double[] xy, double angdeg)
   {
      double x = xy[0];
      double y = xy[1];
      double[] newPoint = new double[2];
      double angRad = Math.toRadians(angdeg);
      newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
      newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);

      return newPoint;
   }

   private void setUpCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double[] centerPoint, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      AppearanceDefinition app = cinderBlockAppearance;

      double xCenter = centerPoint[0];
      double yCenter = centerPoint[1];

      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      location.getTranslation().set(new Vector3D(xCenter, yCenter, cinderBlockHeight / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3D(location,
                                                                                                 cinderBlockLength + overlapToPreventGaps,
                                                                                                 cinderBlockWidth + overlapToPreventGaps,
                                                                                                 cinderBlockHeight + overlapToPreventGaps),
                                                                                       app);
      combinedTerrainObject.addTerrainObject(newBox);

   }

}
