package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.HashMap;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.common.DiscoveryStatus;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.common.Guid.GuidPrefix;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.participant.ParticipantDiscoveryInfo;
import us.ihmc.pubsub.participant.ParticipantListener;
import us.ihmc.robotDataLogger.Announcement;

public class DataConsumerParticipant
{
   private class LeaveListener implements ParticipantListener
   {
      @Override
      public void onParticipantDiscovery(Participant participant, ParticipantDiscoveryInfo info)
      {
         if(info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
         {
            lock.lock();
            Announcement removed = announcements.remove(info.getGuid().getGuidPrefix());
            
            lock.unlock();
         }
      }
      
   }
   
   private final ReentrantLock lock = new ReentrantLock();
   private final Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private final Participant participant;
   private final HashMap<GuidPrefix, Announcement> announcements = new HashMap<>();
   
   public DataConsumerParticipant(String name) throws IOException
   {
      ParticipantAttributes<?> att = domain.createParticipantAttributes();
      att.setDomainId(LogParticipantSettings.domain);
      att.setLeaseDuration(Time.Infinite);
      att.setName(name);
      participant = domain.createParticipant(att, null);
   }

   
   public static void main(String[] args)
   {
      
   }
   
}
