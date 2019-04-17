package us.ihmc.commonWalkingControlModules.sensors.touchdownDetector;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.touchdownDetector.SufficientTouchdownDetectors;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class SufficientTouchdownDetectorsTest
{
   @Test
   public void test()
   {
      SettableTouchdownDetector touchdownDetector1 = new SettableTouchdownDetector();
      SettableTouchdownDetector touchdownDetector2 = new SettableTouchdownDetector();
      SettableTouchdownDetector touchdownDetector3 = new SettableTouchdownDetector();

      SufficientTouchdownDetectors touchdownDetectors = new SufficientTouchdownDetectors();
      touchdownDetectors.addTouchdownDetector(touchdownDetector1);
      touchdownDetectors.addTouchdownDetector(touchdownDetector2);
      touchdownDetectors.addTouchdownDetector(touchdownDetector3);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());

      touchdownDetector1.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertTrue(touchdownDetectors.hasTouchedDown());

      touchdownDetector2.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertTrue(touchdownDetectors.hasTouchedDown());

      touchdownDetector3.setHasTouchedDown(true);

      touchdownDetectors.update();
      assertTrue(touchdownDetectors.hasTouchedDown());

      touchdownDetector2.setHasTouchedDown(false);

      touchdownDetectors.update();
      assertTrue(touchdownDetectors.hasTouchedDown());

      touchdownDetector1.setHasTouchedDown(false);
      touchdownDetector3.setHasTouchedDown(false);

      touchdownDetectors.update();
      assertFalse(touchdownDetectors.hasTouchedDown());
   }
}
