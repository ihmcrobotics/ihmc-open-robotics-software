package us.ihmc.avatar.environments;

import org.apache.commons.lang3.NotImplementedException;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.Fiducial;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionEnvironmentTools;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

public class PhaseOneDemoEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double WALL_WIDTH = ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX();
   private static final double WALL_DEPTH = 0.05;
   private static final double WALL_HEIGHT = 2.4384;
   private static final double efpRegionPenetrationThickness = 0.02;

   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   private final List<PlanarRegionsList> planarRegionsLists = new ArrayList<>();
   private final List<AppearanceDefinition> appearances = new ArrayList<>();

   private final PlanarRegionsList debrisRegions = new PlanarRegionsList();

   private final Point3D pullDoorLocation = new Point3D(10.0, -0.5 * ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0);
   private final Point3D pushDoorLocation = new Point3D(7.5, -0.5 * ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0);
   private final Point3D stairsLocation = new Point3D(13.0, 0.0, 0.0);
   private final double doorYaw = 0.5 * Math.PI;

   public enum StartingLocation
   {
      STARTING_BLOCK(-2.0, -1.0, 0.3, Math.toRadians(20.0)),
      IN_FRONT_OF_PLATFORM(0.7, 0.0, 0.0, 0.0),
      DEBRIS_PLATFORM(3.0, 0.0, 0.575, 0.0),
      PUSH_DOOR(6.3, 0.0, 0.0, 0.0),
      PULL_DOOR(8.8, 0.0, 0.0, 0.0),
      STAIRS(11.75, 0.0, 0.0, 0.0);

      private final Pose3D startingPose = new Pose3D();

      StartingLocation(double x, double y, double z, double yaw)
      {
         startingPose.set(x, y, z, yaw, 0.0, 0.0);
      }

      public Pose3D getPose()
      {
         return startingPose;
      }
   }

   public PhaseOneDemoEnvironment(boolean pushDoor, boolean pullDoor, boolean debris, boolean barrel, boolean stairs, boolean cinderBlockField)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      if (pushDoor)
         createPushDoor();
      if (pullDoor)
         createPullDoor();
      if (barrel)
         createBarrel();
      if (stairs)
         createStairs();
      if (cinderBlockField)
         createCinderBlockField(debris);

      addGroundRegion();

      PlanarRegionsList[] planarRegionsList = this.planarRegionsLists.toArray(new PlanarRegionsList[0]);
      AppearanceDefinition[] appearances = this.appearances.toArray(new AppearanceDefinition[0]);

      PlanarRegionEnvironmentTools.addRegionsToEnvironment(combinedTerrainObject, planarRegionsList, appearances, efpRegionPenetrationThickness);
   }

   private void addGroundRegion()
   {
      ConvexPolygon2D groundPolygon1 = new ConvexPolygon2D();
      groundPolygon1.addVertex(-3.0, 2.5);
      groundPolygon1.addVertex(-3.0, -2.5);
      groundPolygon1.addVertex(-0.5, -2.5);
      groundPolygon1.addVertex(-0.5, 2.5);
      groundPolygon1.addVertex(1.25, -0.5);
      groundPolygon1.addVertex(1.25, 0.5);
      groundPolygon1.update();
      PlanarRegion groundRegion1 = new PlanarRegion(new RigidBodyTransform(), groundPolygon1);
      addRegions(new PlanarRegionsList(groundRegion1), YoAppearance.LightGray());

      ConvexPolygon2D groundPolygon2 = new ConvexPolygon2D();
      groundPolygon2.addVertex(4.5, -1.0);
      groundPolygon2.addVertex(4.5, 1.0);
      groundPolygon2.addVertex(6.0, 2.0);
      groundPolygon2.addVertex(6.0, -2.0);
      groundPolygon2.addVertex(9.0, -1.0);
      groundPolygon2.addVertex(9.0, 1.0);
      groundPolygon2.update();
      PlanarRegion groundRegion2 = new PlanarRegion(new RigidBodyTransform(), groundPolygon2);
      addRegions(new PlanarRegionsList(groundRegion2), YoAppearance.LightGray());
      
      ConvexPolygon2D groundPolygon3 = new ConvexPolygon2D();
      groundPolygon3.addVertex(9.0, -1.0);
      groundPolygon3.addVertex(9.0, 1.0);
      groundPolygon3.addVertex(18.0, -1.0);
      groundPolygon3.addVertex(18.0, 1.0);
      groundPolygon3.update();
      PlanarRegion groundRegion3 = new PlanarRegion(new RigidBodyTransform(), groundPolygon3);
      addRegions(new PlanarRegionsList(groundRegion3), YoAppearance.LightGray());

      PlanarRegionsListGenerator wallRegionsGenerator = new PlanarRegionsListGenerator();
      double wallRegionWidth = 1.0;
      double wallRegionHeight = 0.9;
      double wallSeparationWidth = 1.4;
      wallRegionsGenerator.translate(1.0 + 0.5 * wallRegionWidth, 0.5 * wallSeparationWidth + 0.5 * wallRegionWidth, 0.0);
      wallRegionsGenerator.addCubeReferencedAtBottomMiddle(wallRegionWidth, wallRegionWidth, wallRegionHeight);
      wallRegionsGenerator.identity();
      wallRegionsGenerator.translate(1.0 + 0.5 * wallRegionWidth, -0.5 * wallSeparationWidth - 0.5 * wallRegionWidth, 0.0);
      wallRegionsGenerator.addCubeReferencedAtBottomMiddle(wallRegionWidth, wallRegionWidth, wallRegionHeight);
      addRegions(wallRegionsGenerator.getPlanarRegionsList(), YoAppearance.DarkGray());
   }

   private void createPushDoor()
   {
      Vector3D offset = new Vector3D(ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0, 0.0);
      new AxisAngle(doorYaw, 0.0, 0.0).transform(offset);
      pushDoorLocation.add(offset);

      ContactableDoorRobot door = new ContactableDoorRobot("pushDoorRobot", pushDoorLocation, doorYaw, Fiducial.FIDUCIAL50);
      door.getPinJoint().setQ(Math.PI);

      contactableRobots.add(door);
      door.createAvailableContactPoints(0, 15, 15, 0.02, true);

      RigidBodyTransform wall1Transform = new RigidBodyTransform();
      wall1Transform.getTranslation().set(pushDoorLocation);
      wall1Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall1Transform.appendTranslation(0.5 * WALL_WIDTH, 0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      RigidBodyTransform wall2Transform = new RigidBodyTransform();
      wall2Transform.getTranslation().set(pushDoorLocation);
      wall2Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall2Transform.appendTranslation(-ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0, 0.0);
      wall2Transform.appendTranslation(-0.5 * WALL_WIDTH, -0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      combinedTerrainObject.addRotatableBox(wall1Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
      combinedTerrainObject.addRotatableBox(wall2Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
   }

   private void createPullDoor()
   {
      ContactableDoorRobot pullDoor = new ContactableDoorRobot("pullDoorRobot", pullDoorLocation, doorYaw, Fiducial.FIDUCIAL150);

      contactableRobots.add(pullDoor);
      pullDoor.createAvailableContactPoints(0, 15, 15, 0.02, true);

      RigidBodyTransform wall1Transform = new RigidBodyTransform();
      wall1Transform.getTranslation().set(pullDoorLocation);
      wall1Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall1Transform.appendTranslation(- 0.5 * WALL_WIDTH, - 0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      RigidBodyTransform wall2Transform = new RigidBodyTransform();
      wall2Transform.getTranslation().set(pullDoorLocation);
      wall2Transform.getRotation().setYawPitchRoll(doorYaw, 0.0, 0.0);
      wall2Transform.appendTranslation(ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.0, 0.0);
      wall2Transform.appendTranslation(0.5 * WALL_WIDTH, 0.5 * WALL_DEPTH, 0.5 * WALL_HEIGHT); // referenced at middle

      combinedTerrainObject.addRotatableBox(wall1Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
      combinedTerrainObject.addRotatableBox(wall2Transform, WALL_WIDTH, WALL_DEPTH, WALL_HEIGHT, YoAppearance.Bisque());
   }

   private void createCinderBlockField(boolean addDebris)
   {
      RigidBodyTransform startingBlockTransform = new RigidBodyTransform();
      startingBlockTransform.getTranslation().set(-2.0, -1.0, 0.0);
      startingBlockTransform.getRotation().setYawPitchRoll(Math.toRadians(20.0), 0.0, 0.0);

      addRegions(BehaviorPlanarRegionEnvironments.generateStartingBlockRegions(startingBlockTransform), YoAppearance.Grey());
      addRegions(BehaviorPlanarRegionEnvironments.createRoughUpAndDownStepsWithFlatTop(false), YoAppearance.Grey());

      if (addDebris)
      {
         PlanarRegionsList debrisRegions = DataSetIOTools.loadDataSet(DataSetName._20200624_105955_FBDemoDebris_Medium).getPlanarRegionsList();
         RigidBodyTransformGenerator debrisRegionsTransformGenerator = new RigidBodyTransformGenerator();
         debrisRegionsTransformGenerator.rotate(0.5 * Math.PI, Axis3D.Z);
         debrisRegionsTransformGenerator.translate(-0.8, -2.0, 0.6);
         RigidBodyTransform debrisRegionsTransform = debrisRegionsTransformGenerator.getRigidBodyTransformCopy();

         int indexToExcludeForMediumSizedDebris = 0;
         for (int i = 0; i < debrisRegions.getNumberOfPlanarRegions(); i++)
         {
            if (i != indexToExcludeForMediumSizedDebris)
            {
               PlanarRegion debrisRegion = debrisRegions.getPlanarRegion(i);
               debrisRegion.applyTransform(debrisRegionsTransform);
               this.debrisRegions.addPlanarRegion(debrisRegion);
            }
         }

         addRegionGraphics(this.debrisRegions, YoAppearance.DarkGray());
      }
   }

   private void addRegions(PlanarRegionsList regionsToAdd, AppearanceDefinition appearance)
   {
      planarRegionsLists.add(regionsToAdd);
      appearances.add(appearance);
   }

   private void addRegionGraphics(PlanarRegionsList planarRegionsToAdd, AppearanceDefinition appearance)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();

      for (int i = 0; i < planarRegionsToAdd.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsToAdd.getPlanarRegion(i);
         double thickness = Math.max(efpRegionPenetrationThickness, 1e-4);
         Graphics3DObjectTools.addPlanarRegion(graphics3DObject, planarRegion, thickness, appearance);
      }

      combinedTerrainObject.addStaticLinkGraphics(graphics3DObject);
   }

   public PlanarRegionsList getEnvironmentRegions()
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

   public PlanarRegionsList getDebrisRegions()
   {
      return debrisRegions;
   }

   public PlanarRegionsList getEnvironmentWithDebrisRegions()
   {
      PlanarRegionsList environmentWithoutDebrisRegions = getEnvironmentRegions();
      PlanarRegionsList debrisRegions = getDebrisRegions();
      PlanarRegionsList environmentWithDebrisRegions = new PlanarRegionsList();

      for (int i = 0; i < environmentWithoutDebrisRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = environmentWithoutDebrisRegions.getPlanarRegion(i);
         region.setRegionId(environmentWithDebrisRegions.getNumberOfPlanarRegions());
         environmentWithDebrisRegions.addPlanarRegion(region);
      }
      for (int i = 0; i < debrisRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = debrisRegions.getPlanarRegion(i);
         region.setRegionId(environmentWithDebrisRegions.getNumberOfPlanarRegions());
         environmentWithDebrisRegions.addPlanarRegion(region);
      }

      return environmentWithDebrisRegions;
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

      generator.translate(- 0.5 * stepDepth + 0.5 * topPlatformLength, 0.0, 0.0);
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
}
