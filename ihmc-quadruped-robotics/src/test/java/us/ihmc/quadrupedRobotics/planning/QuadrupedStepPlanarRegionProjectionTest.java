package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepPlanarRegionProjectionTest
{
   @Test
   public void testSimpleProjection()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      QuadrupedStepPlanarRegionProjection projectionCalculator = new QuadrupedStepPlanarRegionProjection(registry);
      double epsilon = 1e-10;

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);
      double minimumDistanceToRegionEdge = projectionCalculator.getMinimumDistanceToRegionEdge();

      double height = 0.5;
      double width = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, height);
      generator.addRectangle(width, width);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      PlanarRegionsListCommand planarRegionsListCommand = createCommand(planarRegionsList);

      // test step is unmodified before regions have been set
      FramePoint3D goalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.7, -0.8, 0.0);
      FramePoint3D expectedProjectedPoint = new FramePoint3D(goalPosition);
      projectionCalculator.project(goalPosition, RobotQuadrant.FRONT_LEFT);
      Assertions.assertTrue(goalPosition.epsilonEquals(expectedProjectedPoint, epsilon));

      // set planar regions
      projectionCalculator.handlePlanarRegionsListCommand(planarRegionsListCommand);

      // test middle of rectangle
      goalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      expectedProjectedPoint = new FramePoint3D(goalPosition);
      projectionCalculator.project(goalPosition, RobotQuadrant.FRONT_LEFT);
      Assertions.assertTrue(goalPosition.epsilonEquals(expectedProjectedPoint, epsilon));

      // test edge of rectangle
      goalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), width * 0.499, 0.0, 0.0);
      expectedProjectedPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * width - minimumDistanceToRegionEdge, 0.0, 0.0);
      projectionCalculator.project(goalPosition, RobotQuadrant.FRONT_LEFT);
      Assertions.assertTrue(goalPosition.epsilonEquals(expectedProjectedPoint, epsilon));

      // test that a step just outside is projected inside the rectangle
      goalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.8 * width, 0.0);
      expectedProjectedPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.5 * width - minimumDistanceToRegionEdge, 0.0);
      projectionCalculator.project(goalPosition, RobotQuadrant.FRONT_LEFT);
      Assertions.assertTrue(goalPosition.epsilonEquals(expectedProjectedPoint, epsilon));

      // resets association of front-left step with region
      projectionCalculator.completedStep(RobotQuadrant.FRONT_LEFT);

      // test that step outside region is unmodified after clearing region association
      goalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.8 * width, 0.0);
      expectedProjectedPoint = new FramePoint3D(goalPosition);
      projectionCalculator.project(goalPosition, RobotQuadrant.FRONT_LEFT);
      Assertions.assertTrue(goalPosition.epsilonEquals(expectedProjectedPoint, epsilon));
   }

   private static PlanarRegionsListCommand createCommand(PlanarRegionsList planarRegionsList)
   {
      PlanarRegionsListCommand planarRegionsListCommand = new PlanarRegionsListCommand();
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      planarRegionsListCommand.setFromMessage(planarRegionsListMessage);
      return planarRegionsListCommand;
   }
}
