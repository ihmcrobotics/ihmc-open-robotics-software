package us.ihmc.simulationconstructionset.physics.collision.gdx;

import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(1000.0);
   }
}
