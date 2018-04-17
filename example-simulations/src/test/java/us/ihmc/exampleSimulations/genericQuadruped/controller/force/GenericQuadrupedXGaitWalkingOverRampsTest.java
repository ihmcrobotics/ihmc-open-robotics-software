package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkingOverRampsTest;

import java.io.IOException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitWalkingOverRampsTest extends QuadrupedXGaitWalkingOverRampsTest
{
   @Override
   public double getDesiredWalkingVelocity()
   {
      return 0.75;
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingDownSlope() throws IOException
   {
      super.testWalkingDownSlope(new InitialWalkDownSlopePosition());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2200000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      super.testWalkingOverShallowRamps(0.575);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 980000)
   public void testWalkingUpSlope() throws IOException
   {
      super.testWalkingUpSlope(new InitialWalkUpSlopePosition());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      super.testWalkingOverAggressiveRamps(0.575);
   }

   private class InitialWalkDownSlopePosition extends GenericQuadrupedDefaultInitialPosition
   {
      @Override
      public Point3D getInitialBodyPosition()
      {
         return new Point3D(0.0, 0.0, 0.05);
      }

      @Override
      public QuaternionReadOnly getInitialBodyOrientation()
      {
         return new Quaternion(0.0, 0.2, 0.0);
      }
   }

   private class InitialWalkUpSlopePosition extends GenericQuadrupedDefaultInitialPosition
   {
      @Override
      public Point3D getInitialBodyPosition()
      {
         return new Point3D(0.0, 0.0, 0.1);
      }

      @Override
      public QuaternionReadOnly getInitialBodyOrientation()
      {
         return new Quaternion(0.0, -0.1, 0.0);
      }
   }
}
