package us.ihmc.simulationconstructionset.physics.collision.gdx;

import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(1000.0);
   }
}
