package us.ihmc.simulationconstructionset.physics.collision.gdx;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(1000.0);
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 300000)
   public void collisionMask_hit()
   {
      super.collisionMask_hit();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 300000)
   public void testBoxBarelyCollisions()
   {
      super.testBoxBarelyCollisions();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testBoxCloseButNoCollisions()
   {
      super.testBoxCloseButNoCollisions();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSmallBox()
   {
      super.testSmallBox();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testUnitBox()
   {
      super.testUnitBox();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void checkCollisionShape_offset()
   {
      super.checkCollisionShape_offset();
   }
}
