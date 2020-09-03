package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSphereRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class HeavyBallOnTableEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D terrain;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();
   private final ContactController contactController = new ContactController("HeavyBall");

   private final double ballRadius = 0.25;
   private final double ballMass = 25.0;
   private final ContactableSphereRobot ballRobot;

   public HeavyBallOnTableEnvironment()
   {
      terrain = DefaultCommonAvatarEnvironment.setUpGround("Ground");

      double centerX = 0.6;
      double centerY = 0.0;
      double halfSize = 0.6 / 2.0;
      double edgeThick = 0.05;
      double edgeHeight = 0.10;

      double xStart = centerX - halfSize;
      double yStart = centerY - halfSize;
      double xEnd = centerX + halfSize;
      double yEnd = centerY + halfSize;

      terrain.addBox(xStart, yStart, xEnd, yEnd, 0.8, YoAppearance.Brown());
      terrain.addBox(xStart, yStart, xStart + edgeThick, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());
      terrain.addBox(xEnd - edgeThick, yStart, xEnd, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());

      terrain.addBox(xStart + edgeThick, yStart, xEnd - edgeThick, yStart + edgeThick, 0.8 + edgeHeight, YoAppearance.Brown());
      terrain.addBox(xStart + edgeThick, yEnd - edgeThick, xEnd - edgeThick, yEnd, 0.8 + edgeHeight, YoAppearance.Brown());

      ballRobot = new ContactableSphereRobot("DatBall", ballRadius, ballMass, YoAppearance.BlackMetalMaterial());
      Point3D[] contactPointsOffset = SpiralBasedAlgorithm.generatePointsOnSphere(ballRadius, 500);
      for (int i = 0; i < contactPointsOffset.length; i++)
      {
         Point3D contactPointOffset = contactPointsOffset[i];
         GroundContactPoint gc = new GroundContactPoint("ballGC_" + i, new Vector3D(contactPointOffset), ballRobot);
         ballRobot.getRootJoints().get(0).addGroundContactPoint(gc);
      }
      LinearGroundContactModel ballGCModel = new LinearGroundContactModel(ballRobot, ballRobot.getRobotsYoRegistry());
      ballGCModel.setGroundProfile3D(terrain);
      ballRobot.setGroundContactModel(ballGCModel);
      ballRobot.setPosition(centerX, centerY, 0.8 + ballRadius + 0.1);
      ballRobot.createAvailableContactPoints(1, 10, 0.1, true);
      ballRobot.setController(contactController);

      contactController.addContactable(ballRobot);
      contactController.setContactParameters(1500.0, 50.0, 0.9, 0.8);

      environmentRobots.add(ballRobot);
   }

   public ContactableSphereRobot getBallRobot()
   {
      return ballRobot;
   }

   public double getBallRadius()
   {
      return ballRadius;
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public CombinedTerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
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
      contactController.addContactPoints(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
