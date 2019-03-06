package us.ihmc.commonWalkingControlModules.sensors.touchdownDetector;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.touchdownDetector.NecessaryTouchdownDetectors;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class NecessaryTouchdownDetectorsTest
{
   @Test
   public void test()
   {
      SettableTouchdownDetector touchdownDetector1 = new SettableTouchdownDetector();
      SettableTouchdownDetector touchdownDetector2 = new SettableTouchdownDetector();
      SettableTouchdownDetector touchdownDetector3 = new SettableTouchdownDetector();

      NecessaryTouchdownDetectors touchdownDetectors = new NecessaryTouchdownDetectors();
      touchdownDetectors.addTouchdownDetector(touchdownDetector1);
      touchdownDetectors.addTouchdownDetector(touchdownDetector2);
      touchdownDetectors.addTouchdownDetector(touchdownDetector3);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());

      touchdownDetector1.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());

      touchdownDetector2.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());

      touchdownDetector3.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertTrue(touchdownDetectors.hasTouchedDown());

      touchdownDetector2.setHasTouchedDown(false);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());
   }
}
