package us.ihmc.perception.vio;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class VIOTrackingSessionTest
{
   @Test
   void waitsForARequestAndRejectsDataFromBeforeTheRestart()
   {
      VIOTrackingSession trackingSession = new VIOTrackingSession();
      long requestTime = 1_793_212_000_000_000_000L;
      assertFalse(trackingSession.acceptsDataAt(requestTime));
      assertEquals(0, trackingSession.takePendingRestartRequest());

      trackingSession.requestTrackingRestart(requestTime);
      assertEquals(requestTime, trackingSession.getSessionStartTime());
      assertFalse(trackingSession.acceptsDataAt(requestTime + 1));

      long restartRequestToken = trackingSession.takePendingRestartRequest();
      assertNotEquals(0, restartRequestToken);
      assertEquals(0, trackingSession.takePendingRestartRequest());

      long restartTime = requestTime + 100_000_000L;
      assertTrue(trackingSession.finishTrackingRestart(restartRequestToken, true, restartTime));
      assertFalse(trackingSession.acceptsDataAt(requestTime));
      assertTrue(trackingSession.acceptsDataAt(restartTime + 1));

      trackingSession.markTrackingValid();
      assertEquals(VIOTrackingSession.Phase.WAITING_FOR_ANCHOR, trackingSession.getPhase());
      trackingSession.markAnchored();
      assertEquals(VIOTrackingSession.Phase.RUNNING, trackingSession.getPhase());
   }

   @Test
   void newerRequestSupersedesARestartInFlightAndAFailedRestartStaysPaused()
   {
      VIOTrackingSession trackingSession = new VIOTrackingSession();
      long requestTime = 10_000_000_000L;
      trackingSession.requestTrackingRestart(requestTime);
      String firstSessionId = trackingSession.getSessionId();
      long firstRequestToken = trackingSession.takePendingRestartRequest();

      trackingSession.requestTrackingRestart(requestTime + 1);
      assertNotEquals(firstSessionId, trackingSession.getSessionId());
      assertFalse(trackingSession.finishTrackingRestart(firstRequestToken, true, requestTime + 2));
      assertFalse(trackingSession.isTracking());

      long secondRequestToken = trackingSession.takePendingRestartRequest();
      assertTrue(trackingSession.finishTrackingRestart(secondRequestToken, false, requestTime + 3));
      assertEquals(VIOTrackingSession.Phase.ERROR, trackingSession.getPhase());
      assertFalse(trackingSession.acceptsDataAt(requestTime + 4));

      trackingSession.requestTrackingRestart(requestTime + 5);
      trackingSession.finishTrackingRestart(trackingSession.takePendingRestartRequest(), true, requestTime + 6);
      assertTrue(trackingSession.stopForCameraReopen());
      assertFalse(trackingSession.isTracking());
      assertEquals(Long.MAX_VALUE, trackingSession.getSessionStartTime());
   }
}
