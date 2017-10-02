package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSphereRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.BoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class StaticFootstepPlanningEnvironment implements CommonAvatarEnvironmentInterface
{
   public enum GoalMarkerLocation
   {
      ORIGIN(0.0, 0.0, 2.0), TOP_OF_STAIRS(STAIRS_START_X + 1.5, STAIRS_START_Y, 1.5);

      private final Point3D markerLocation;

      GoalMarkerLocation(double x, double y, double z)
      {
         markerLocation = new Point3D(x, y, z);
      }

      public Point3D getMarkerLocation()
      {
         return markerLocation;
      }
   }

   private static final double ROBOT_AREA_WIDTH_IN_METERS = 7.3152;
   private static final double ROBOT_AREA_LENGTH_IN_METERS = 21.9456;

   private static final double SHORT_CINDER_BLOCK_FIELD_START_X = 1.0;
   private static final double SHORT_CINDER_BLOCK_FIELD_START_Y = 2.0;

   private static final double RAMPS_WITH_STEPPING_STONES_START_X = 4.0;
   private static final double RAMPS_WITH_STEPPING_STONES_START_Y = 2.0;

   private static final double ROCK_FIELD_START_X = 7.0;
   private static final double ROCK_FIELD_START_Y = -3.0;
   private static final double ROCK_FIELD_LENGTH = 8.0;
   private static final double ROCK_FIELD_WIDTH = 2.0;

   private static final double STAIRS_START_X = 1.0;
   private static final double STAIRS_START_Y = -1.5;

   private static final double SHORT_BARRIER_START_X = 3.3;
   private static final double SHORT_BARRIER_START_Y = -2.8;

   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D("StaticFootstepPlanningEnvironment");
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   private final ContactableSphereRobot goalMarkerRobot = new ContactableSphereRobot("FootstepPlannerGoalMarker", 0.1, 1.0, YoAppearance.Green());

   public StaticFootstepPlanningEnvironment()
   {
      setupGround();
      addShortCinderBlockField(SHORT_CINDER_BLOCK_FIELD_START_X, SHORT_CINDER_BLOCK_FIELD_START_Y);
      addRampsWithSteppingStones(RAMPS_WITH_STEPPING_STONES_START_X, RAMPS_WITH_STEPPING_STONES_START_Y);
      addRocks(ROCK_FIELD_START_X, ROCK_FIELD_START_Y, ROCK_FIELD_LENGTH, ROCK_FIELD_WIDTH);
      addStairs(STAIRS_START_X, STAIRS_START_Y);
      addShortBarrier(SHORT_BARRIER_START_X, SHORT_BARRIER_START_Y);

      addGoalMarkerRobot();
   }

   private void setupGround()
   {

      double robotStartingPointX = 1.0;

      BoxTerrainObject groundTerrainObject = new BoxTerrainObject(-robotStartingPointX, -ROBOT_AREA_WIDTH_IN_METERS / 2,
            ROBOT_AREA_LENGTH_IN_METERS - robotStartingPointX, ROBOT_AREA_WIDTH_IN_METERS / 2,
            -0.5, 0.0, YoAppearance.DarkGray());

      combinedTerrainObject3D.addTerrainObject(groundTerrainObject);
   }

   private void addShortCinderBlockField(double startDistanceX, double startDistanceY)
   {
      CombinedTerrainObject3D shortCinderBlockField = DefaultCommonAvatarEnvironment.setUpShortCinderBlockField("ShortCinderBlockField",
           0.0, startDistanceX, startDistanceY);
      combinedTerrainObject3D.addTerrainObject(shortCinderBlockField);
   }

   private void addRampsWithSteppingStones(double startDistanceX, double startDistanceY)
   {
      AppearanceDefinition color = YoAppearance.LightGray();

      // ramp up and landing
      double rampLength = 3.0;
      double rampWidth = 3.0;
      double rampHeight = 0.3;
      double rampLandingLength = 1.0;

      combinedTerrainObject3D.addRamp(startDistanceX, startDistanceY - rampWidth/2, startDistanceX + rampLength, startDistanceY + rampWidth/2,
            rampHeight, color);
      startDistanceX += rampLength;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - rampWidth/2, startDistanceX + rampLandingLength, startDistanceY + rampWidth/2,
            rampHeight, color);
      double endOfRampLanding = startDistanceX + rampLandingLength;

      // simple stepping stones, centered at x=-0.75m
      double steppingStoneOffsetFromCenter = 0.75;
      double steppingStoneLength = 0.5;
      double steppingStoneWidth = 0.5;

      startDistanceX = endOfRampLanding;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY + steppingStoneOffsetFromCenter - steppingStoneWidth, startDistanceX + steppingStoneLength,
            startDistanceY + steppingStoneOffsetFromCenter, rampHeight, color);
      startDistanceX += steppingStoneLength;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY + steppingStoneOffsetFromCenter, startDistanceX + steppingStoneLength,
            startDistanceY + steppingStoneOffsetFromCenter + steppingStoneWidth, rampHeight, color);
      startDistanceX += steppingStoneLength;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY + steppingStoneOffsetFromCenter - steppingStoneWidth, startDistanceX + steppingStoneLength,
            startDistanceY + steppingStoneOffsetFromCenter, rampHeight, color);
      startDistanceX += steppingStoneLength;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY + steppingStoneOffsetFromCenter, startDistanceX + steppingStoneLength,
            startDistanceY + steppingStoneOffsetFromCenter + steppingStoneWidth, rampHeight, color);
      startDistanceX += steppingStoneLength;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY + steppingStoneOffsetFromCenter - steppingStoneWidth, startDistanceX + steppingStoneLength,
            startDistanceY + steppingStoneOffsetFromCenter, rampHeight, color);

      double endOfSteppingStones = startDistanceX + steppingStoneLength;

      // qualification stepping stones, centered along x=0.75m
      startDistanceX = endOfRampLanding;

      double firstSteppingStoneXOffset = 0.25;
      startDistanceX += firstSteppingStoneXOffset;
      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - steppingStoneOffsetFromCenter - steppingStoneWidth, startDistanceX + steppingStoneLength,
            startDistanceY - steppingStoneOffsetFromCenter, rampHeight, color);
      startDistanceX += steppingStoneLength;

      double secondSteppingStoneXOffset = 0.0;
      startDistanceX += secondSteppingStoneXOffset;
      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - steppingStoneOffsetFromCenter, startDistanceX + steppingStoneLength,
            startDistanceY - steppingStoneOffsetFromCenter + steppingStoneWidth, rampHeight, color);
      startDistanceX += steppingStoneLength;

      double thirdSteppingStoneXOffset = 0.3;
      startDistanceX += thirdSteppingStoneXOffset;
      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - steppingStoneOffsetFromCenter - steppingStoneWidth, startDistanceX + steppingStoneLength,
            startDistanceY - steppingStoneOffsetFromCenter, rampHeight, color);

      // landing and ramp down
      startDistanceX = endOfSteppingStones;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - rampWidth/2, startDistanceX + rampLandingLength, startDistanceY + rampWidth/2,
            rampHeight, color);
      startDistanceX += rampLandingLength;

      combinedTerrainObject3D.addRamp(startDistanceX + rampLength, startDistanceY - rampWidth/2, startDistanceX, startDistanceY + rampWidth/2,
            rampHeight, color);
   }

   private void addRocks(double startDistanceX, double startDistanceY, double rockFieldLength, double rockFieldWidth)
   {
      Random random = new Random(34782L);
      int numberOfRocksToAdd = 80;
      double maxRockHeightInMeters = 0.2;
      double maxRockWidth = 0.5;

      for (int i = 0; i < numberOfRocksToAdd; i++)
      {
         double centroidHeight = random.nextDouble() * maxRockHeightInMeters;
         Vector3D normal = new Vector3D(0.0, 0.0, 1.0);

         double[] approximateCentroid = new double[2];

         approximateCentroid[0] = startDistanceX + random.nextDouble() * rockFieldLength;
         approximateCentroid[1] = startDistanceY + random.nextDouble() * rockFieldWidth;

         int verticesPerRock = 21;
         double[][] vertices = new double[verticesPerRock][2];

         for (int j = 0; j < verticesPerRock; j++)
         {
            vertices[j][0] = random.nextDouble() * maxRockWidth + approximateCentroid[0] - maxRockWidth / 2.0;
            vertices[j][1] = random.nextDouble() * maxRockWidth + approximateCentroid[1] - maxRockWidth / 2.0;
         }

         DefaultCommonAvatarEnvironment.addRock3D(combinedTerrainObject3D, normal, centroidHeight, vertices);
      }
   }

   private void addStairs(double startDistanceX, double startDistanceY)
   {
      AppearanceDefinition color = YoAppearance.LightGray();

      int numberOfSteps = 4;
      double stairLength = 0.4;
      double stairWidth = 1.5;
      double stairHeight = 0.2;

      for(int i = 1; i < numberOfSteps + 1; i++)
      {
         combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - stairWidth/2, startDistanceX + stairLength, startDistanceY + stairWidth/2,
               stairHeight * i, color);
         startDistanceX += stairLength;
      }

      double platformLength = 1.0;
      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - stairWidth/2, startDistanceX + platformLength, startDistanceY + stairWidth/2,
            stairHeight * numberOfSteps, color);
      startDistanceX += platformLength;

      for(int i = numberOfSteps; i > 0; i--)
      {
         combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - stairWidth/2, startDistanceX + stairLength, startDistanceY + stairWidth/2,
               stairHeight * i, color);
         startDistanceX += stairLength;
      }
   }

   private void addShortBarrier(double startDistanceX, double startDistanceY)
   {
      AppearanceDefinition color = YoAppearance.LightGray();

      double barrierLength = 0.1;
      double barrierWidth = 1.0;
      double barrierHeight = 0.5;

      combinedTerrainObject3D.addBox(startDistanceX, startDistanceY - barrierWidth/2, startDistanceX + barrierLength, startDistanceY + barrierWidth/2,
            barrierHeight, color);
   }

   private void addGoalMarkerRobot()
   {
      goalMarkerRobot.setGravity(0.0);
      environmentRobots.add(goalMarkerRobot);

      YoVariableRegistry robotsYoVariableRegistry = goalMarkerRobot.getRobotsYoVariableRegistry();
      YoEnum<GoalMarkerLocation> goalMarkerLocationYoEnum = YoEnum.create("DesiredGoalMarkerLocation", GoalMarkerLocation.class, robotsYoVariableRegistry);
      goalMarkerLocationYoEnum.addVariableChangedListener(new GoalMarkerLocationUpdater());
      goalMarkerLocationYoEnum.set(GoalMarkerLocation.TOP_OF_STAIRS);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {

   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {

   }

   class GoalMarkerLocationUpdater implements VariableChangedListener
   {
      @Override
      public void notifyOfVariableChange(YoVariable<?> v)
      {
         GoalMarkerLocation goalMarkerLocation = ((YoEnum<GoalMarkerLocation>) v).getEnumValue();
         goalMarkerRobot.setPosition(goalMarkerLocation.getMarkerLocation());
      }
   }
}
