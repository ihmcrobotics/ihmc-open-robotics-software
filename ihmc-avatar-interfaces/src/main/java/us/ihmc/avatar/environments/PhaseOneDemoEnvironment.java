package us.ihmc.avatar.environments;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationConstructionSetTools.util.environments.AdjustableStairsEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.Fiducial;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
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
   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final double WALL_HEIGHT = 2.4384;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   private final Point3D pullDoorLocation = new Point3D(0, -3.5, 0);

   private final Point3D pushDoorLocation = new Point3D(0, -3.5, 0);

   public PhaseOneDemoEnvironment(boolean door, boolean debris, boolean barrel, boolean wakling, boolean stairs)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      if (door)
      {
        // createPushDoor(pushDoorLocation);
         createPullDoor(pullDoorLocation);
      }
      if (barrel)
         createBarrel();
      if (debris)
         createDebris();
      if (wakling)
         createWalkingCourse();
      if (stairs)
         createStairs();

   }

   private void createWalkingCourse()
   {
      combinedTerrainObject.addTerrainObject(setUpCinderBlockFieldActual("walking", -90, 10, 1));
   }

   private void addFullCourseModel(boolean doingValve)
   {
      combinedTerrainObject.getLinkGraphics().identity();

      combinedTerrainObject.getLinkGraphics().addModelFile("models/SCTestBed.obj");
   }

   private void createStairs()
   {
      AdjustableStairsEnvironment environment = new AdjustableStairsEnvironment();
      environment.setStairsParameters(4, 1.016, 0.2286, 0.2921);
      environment.setRailingParameters(0.05, 0.3, 0.05, 0.8128, 2, false);
      environment.setLandingPlatformParameters(1.27, 3, 1.143, 2);
      environment.setCourseAngle(-90);
      environment.setCourseStartDistance(14.2794);
      environment.setCourseOffsetSide(1);

      environment.generateTerrains();
      ArrayList<TerrainObject3D> stairs = ((CombinedTerrainObject3D) environment.getTerrainObject3D()).getTerrainObjects();
      for (TerrainObject3D object : stairs)
      {
         if (object instanceof CombinedTerrainObject3D)
         {
            if (!((CombinedTerrainObject3D) object).getName().contains("ground"))
            {
               combinedTerrainObject.addTerrainObject(object);
            }
         }
      }

   }

   private void createDebris()
   {
      throw new NotImplementedException("Debris not implemented");
   }

   private void createPullDoor(Point3D location)
   {
      ContactableDoorRobot door = new ContactableDoorRobot("PullDoorRobot", location, Fiducial.FIDUCIAL150);
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

      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

   }
   
   
   private void createPushDoor(Point3D location)
   {
      ContactableDoorRobot door = new ContactableDoorRobot("PushDoorRobot", location, Fiducial.FIDUCIAL50);
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

      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

   }

   private void createBarrel()
   {
      throw new NotImplementedException("Barrel not implemented");
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-5.0, -30.0, 5.0, 5.0, -0.05, 0.0, YoAppearance.DarkGray());

      return combinedTerrainObject;
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
