package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.BoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

public class StaticFootstepPlanningEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double ROBOT_AREA_WIDTH_IN_METERS = 7.3152;
   private static final double ROBOT_AREA_LENGTH_IN_METERS = 21.9456;

   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D("StaticFootstepPlanningEnvironment");
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public StaticFootstepPlanningEnvironment()
   {
      setupGround();
      addShortCinderBlockField(2.0, 2.0);
      addRampsWithSteppingStones(5.0 , 2.0);


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

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
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
      contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {

   }
}
