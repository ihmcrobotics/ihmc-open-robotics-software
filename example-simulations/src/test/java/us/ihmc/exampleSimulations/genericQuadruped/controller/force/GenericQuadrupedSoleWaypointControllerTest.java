package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSquaredUpInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSoleWaypointControllerTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class GenericQuadrupedSoleWaypointControllerTest extends QuadrupedSoleWaypointControllerTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      GenericQuadrupedTestFactory testFactory = new GenericQuadrupedTestFactory();
      testFactory.setInitialPosition(new GenericQuadrupedSquaredUpInitialPosition());

      return testFactory;
   }


}
