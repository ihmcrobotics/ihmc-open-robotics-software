package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment.BLOCKTYPE;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DarpaRoboticsChallengeFinalsEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final double WALL_HEIGHT = 2.4384;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public DarpaRoboticsChallengeFinalsEnvironment(boolean door, boolean drill, boolean valve, boolean wakling, boolean stairs)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      addFullCourse3dModel(valve);

      if (door)
         createDoor();
      if (valve)
         createValve();
      if (drill)
         createDrill();
      if (wakling)
         createWalkingCourse();
      if (stairs)
         createStairs();

   }

   private void createWalkingCourse()
   {
      combinedTerrainObject.addTerrainObject(setUpCinderBlockFieldActual("walking", -90, 10, 1));
   }

   private void addFullCourse3dModel(boolean doingValve)
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

   private void createDrill()
   {
      Vector3d tableCenter = new Vector3d(-0.851, -6.833, 1.118);

      double drillHeight = 0.3;
      double drillRadius = 0.03;
      double drillMass = 1.5;
      double forceVectorScale = 1.0 / 500.0;

      RigidBodyTransform initialDrillTransform = new RigidBodyTransform(new AxisAngle4d(), tableCenter);
      ContactableCylinderRobot drillRobot = new ContactableCylinderRobot("drill", initialDrillTransform, drillRadius, drillHeight, drillMass,
            "models/drill.obj");
      final int groundContactGroupIdentifier = 0;
      drillRobot.createAvailableContactPoints(groundContactGroupIdentifier, 30, forceVectorScale, true);
      for (int i = 0; i < 4; i++)
      {
         double angle = i * 2.0 * Math.PI / 4.0;
         double x = 1.5 * drillRadius * Math.cos(angle);
         double y = 1.5 * drillRadius * Math.sin(angle);
         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_drill_" + i, new Vector3d(x, y, 0.0), drillRobot);
         drillRobot.getRootJoints().get(0).addGroundContactPoint(groundContactPoint);
      }

      GroundContactModel groundContactModel = new LinearGroundContactModel(drillRobot, groundContactGroupIdentifier, 1422.0, 150.6, 50.0, 600.0,
            drillRobot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(combinedTerrainObject);
      drillRobot.setGroundContactModel(groundContactModel);

      contactableRobots.add(drillRobot);

      combinedTerrainObject.addBox(-0.777, -6.916, -0.947, -6.746, 1.061, 1.08);

   }

   private void createDoor()
   {
      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", new Point3d());
      contactableRobots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);
   }

   private void createValve()
   {
      double forceVectorScale = 1.0 / 50.0;

      String valveRobotName = "ValveRobot";
      ValveType valveType = ValveType.SMALL_VALVE;
      double x = -0.758;
      double y = -8.515;
      double z = 1.067;
      double yaw_degrees = -135;

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d(x, y, z);
      Quat4d orientation = new Quat4d();

      RotationTools.convertYawPitchRollToQuaternion(Math.toRadians(yaw_degrees), Math.toRadians(0), Math.toRadians(0), orientation);
      valvePose.setPose(position, orientation);

      ContactableValveRobot valve = new ContactableValveRobot(valveRobotName, valveType, 0.5, valvePose);

      valve.createValveRobot();
      valve.createAvailableContactPoints(1, 30, forceVectorScale, true);

      contactableRobots.add(valve);
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-5.0, -30.0, 5.0, 5.0, -0.05, 0.0, YoAppearance.DarkGray());
      combinedTerrainObject.addBox(-1.2192, -0.025, 0, 0.025, WALL_HEIGHT, YoAppearance.Beige());
      combinedTerrainObject.addBox(0.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), -0.025, 1.2192 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(),
            0.025, WALL_HEIGHT, YoAppearance.Beige());
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

   //************************ CINDER BLOCK STUFF CLEAN UP LATER

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
      BLOCKTYPE[][] blockType = new BLOCKTYPE[nBlocksLong][nBlocksWide];
      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            blockHeight[i][j] = -1; // (int) Math.round(Math.random()*4-1);
            blockAngle[i][j] = 0; // (int) Math.round(Math.random()*3)*45;
            blockType[i][j] = BLOCKTYPE.ANGLED;
         }
      }

      blockHeight = new int[][] { { 0, 0, 0, 0, 0, 0 }, { 0, 0, 1, 1, 0, 0 }, { 0, 0, 1, 1, 0, 0 }, { 0, 1, 1, 1, 1, 0 }, { 1, 2, 1, 1, 2, 1 },
            { 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0 } };

      final int NORTH = 0;
      final int SOUTH = 180;
      final int WEST = 90;
      final int EAST = -90;

      blockAngle = new double[][] { { NORTH, EAST, SOUTH, WEST, NORTH, EAST }, { WEST, NORTH, EAST, SOUTH, WEST, NORTH },
            { SOUTH, WEST, NORTH, EAST, SOUTH, WEST }, { EAST, SOUTH, WEST, NORTH, EAST, SOUTH }, { NORTH, EAST, SOUTH, WEST, NORTH, EAST },
            { WEST, NORTH, EAST, SOUTH, WEST, NORTH }, { SOUTH, WEST, NORTH, EAST, SOUTH, WEST } };

      startDistance += cinderBlockLength / 2;

      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            double xCenter = startDistance + i * cinderBlockLength;
            double yCenter = leftRightOffset + (nBlocksWide * cinderBlockLength) / 2 - j * cinderBlockLength - cinderBlockLength / 2;
            double[] point = { xCenter, yCenter };
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
      double[] xySupportRotatedOffset = rotateAroundOrigin(new double[] { (cinderBlockLength - rampRise) / 2, 0 }, yawDegrees);
      blockSupportLocation.setTranslation(new Vector3d(xCenter + xySupportRotatedOffset[0], yCenter + xySupportRotatedOffset[1], rampRise / 2
            + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(blockSupportLocation, rampRise, cinderBlockLength, rampRise),
            cinderBlockAppearance);
      combinedTerrainObject.addTerrainObject(newBox);

      double xOffset = 0, yOffset = cinderBlockWidth / 2;
      double[] xyRotated1 = rotateAroundOrigin(new double[] { xOffset, yOffset }, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] { xOffset, -yOffset }, yawDegrees);
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
      location.setTranslation(new Vector3d(xCenter, yCenter, zCenter + numberFlatSupports * cinderBlockHeight));
      RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth,
            cinderBlockHeight), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpCinderBlockSquare(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double xOffset = 0, yOffset = cinderBlockWidth / 2.0;
      double[] xyRotated1 = rotateAroundOrigin(new double[] { xOffset, yOffset }, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] { xOffset, -yOffset }, yawDegrees);

      setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
      setUpCinderBlock(combinedTerrainObject, xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);

      if (numberFlatSupports > 0)
         setUpCinderBlockSquare(combinedTerrainObject, xCenter, yCenter, numberFlatSupports - 1, yawDegrees + 90);
   }

   private void setUpCinderBlock(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double[] centerPoint = { xCenter, yCenter };
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

      location.setTranslation(new Vector3d(xCenter, yCenter, cinderBlockHeight / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableCinderBlockTerrainObject newBox = new RotatableCinderBlockTerrainObject(new Box3d(location, cinderBlockLength + overlapToPreventGaps,
            cinderBlockWidth + overlapToPreventGaps, cinderBlockHeight + overlapToPreventGaps), app);
      combinedTerrainObject.addTerrainObject(newBox);

   }

}
