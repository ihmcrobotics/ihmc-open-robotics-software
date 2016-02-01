package us.ihmc.multicastLogDataProtocol.broadcast;

public interface LogBroadcastListener
{
   public void logSessionCameOnline(AnnounceRequest description);
   public void logSessionWentOffline(AnnounceRequest description);
   
}
