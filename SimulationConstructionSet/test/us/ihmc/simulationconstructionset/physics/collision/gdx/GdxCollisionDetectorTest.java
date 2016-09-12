package us.ihmc.simulationconstructionset.physics.collision.gdx;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;

public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(new YoVariableRegistry("Dummy"), 1000);
   }
}
