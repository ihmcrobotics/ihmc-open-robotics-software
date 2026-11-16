package us.ihmc.perception.vio;

import java.util.UUID;

/**
 * State of the VIO tracking session in the autonomy process, independent of the camera. Each tracking restart starts a new
 * session with a new ID, which tags the poses sent during it. Callers serialize its use.
 */
public class VIOTrackingSession
{
   public enum Phase
   {
      WAITING_FOR_START,
      RESTART_REQUESTED,
      RESTARTING,
      WAITING_FOR_TRACKING,
      WAITING_FOR_ANCHOR,
      RUNNING,
      ERROR
   }

   private Phase phase = Phase.WAITING_FOR_START;
   /** Identifies the newest restart request, so the result of a superseded restart is ignored. */
   private long restartRequestToken;
   private long sessionStartTime = Long.MAX_VALUE;
   private String sessionId = "waiting";

   /** Starts a new session and asks the camera thread to restart tracking. A newer request supersedes one in flight. */
   public void requestTrackingRestart(long requestTime)
   {
      restartRequestToken++;
      sessionStartTime = requestTime;
      sessionId = UUID.randomUUID().toString();
      phase = Phase.RESTART_REQUESTED;
   }

   /** On the camera thread: the token of a pending restart request, or 0 if there is none. */
   public long takePendingRestartRequest()
   {
      if (phase != Phase.RESTART_REQUESTED)
         return 0;
      phase = Phase.RESTARTING;
      return restartRequestToken;
   }

   /** Records the result of a tracking restart. Returns false if a newer request has superseded it. */
   public boolean finishTrackingRestart(long requestToken, boolean restartSucceeded, long restartTime)
   {
      if (requestToken != restartRequestToken || phase != Phase.RESTARTING)
         return false;
      sessionStartTime = restartTime;
      phase = restartSucceeded ? Phase.WAITING_FOR_TRACKING : Phase.ERROR;
      return true;
   }

   /** Whether data timestamped at {@code timestamp} belongs to the current session while it is tracking. */
   public boolean acceptsDataAt(long timestamp)
   {
      return timestamp >= sessionStartTime && isTracking();
   }

   public boolean isTracking()
   {
      return phase == Phase.WAITING_FOR_TRACKING || phase == Phase.WAITING_FOR_ANCHOR || phase == Phase.RUNNING;
   }

   public void markTrackingValid()
   {
      if (phase == Phase.WAITING_FOR_TRACKING)
         phase = Phase.WAITING_FOR_ANCHOR;
   }

   public void markAnchored()
   {
      if (isTracking())
         phase = Phase.RUNNING;
   }

   /** A reopened camera must never continue using the previous camera world's anchor. Returns whether a session was stopped. */
   public boolean stopForCameraReopen()
   {
      if (!isTracking())
         return false;
      phase = Phase.WAITING_FOR_START;
      sessionStartTime = Long.MAX_VALUE;
      return true;
   }

   public Phase getPhase()
   {
      return phase;
   }

   public String getSessionId()
   {
      return sessionId;
   }

   /** Wall time the current session started, or {@link Long#MAX_VALUE} before the first one. */
   public long getSessionStartTime()
   {
      return sessionStartTime;
   }
}
