package us.ihmc.commonWalkingControlModules.capturePoint;

import com.google.common.util.concurrent.AtomicDouble;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertEquals;

public class ContactStateManagerTest
{
   @Test
   public void testMaxSpeedUp()
   {
      AtomicDouble time = new AtomicDouble(0.0);
      double minSwingDuration = 0.4;
      double minTransferDuration = 0.05;
      YoRegistry registry = new YoRegistry("test");
      ContactStateManager contactStateManager = new ContactStateManager(time::get, minSwingDuration, minTransferDuration, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double timeAtSTart = 1.7;
      time.set(timeAtSTart);
      double swingDuration = 1.0;
      contactStateManager.initializeForSingleSupport(0.5, swingDuration);

      // run the test at the beginning of swing. Because we know the requested speed up doesn't exceed the min swing duration, and we're doing it at the
      // beginning, we don't have to worry about the speed up factor
      double timeAtCompute = timeAtSTart;
      double setSpeedUpDesired = 0.5;
      time.set(timeAtCompute);
      contactStateManager.updateTimeInState(() -> setSpeedUpDesired, true);

      double expectedTotalTimeRemaining = swingDuration - (timeAtCompute - timeAtSTart);
      assertEquals(expectedTotalTimeRemaining, contactStateManager.getTimeRemainingInCurrentSupportSequence(), 1e-7);
      double adjustedTimeRemaining = contactStateManager.getAdjustedTimeRemainingInCurrentSupportSequence()
                                     - contactStateManager.getExtraTimeAdjustmentForSwing();
      assertEquals(swingDuration - setSpeedUpDesired, adjustedTimeRemaining, 1e-7);

      // advance a little bit through swing. Because we're now doing this further into swing, we can't acheive the full requested amount for the swing
      // speed up. Because of that, we need to compute what the applied swing speed up factor is.
      timeAtCompute += 0.2;
      time.set(timeAtCompute);
      contactStateManager.updateTimeInState(() -> setSpeedUpDesired, true);

      double maxSpeedUpFactor = swingDuration / minSwingDuration;
      expectedTotalTimeRemaining = swingDuration - (timeAtCompute - timeAtSTart);
      double scaledTimeRemaining = expectedTotalTimeRemaining / maxSpeedUpFactor;
      assertEquals(expectedTotalTimeRemaining, contactStateManager.getTimeRemainingInCurrentSupportSequence(), 1e-7);
      adjustedTimeRemaining = contactStateManager.getAdjustedTimeRemainingInCurrentSupportSequence()
                                     - contactStateManager.getExtraTimeAdjustmentForSwing();
      assertEquals(scaledTimeRemaining, adjustedTimeRemaining, 1e-7);
   }


}
