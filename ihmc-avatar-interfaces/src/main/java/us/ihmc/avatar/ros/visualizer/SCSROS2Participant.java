package us.ihmc.avatar.ros.visualizer;

import us.ihmc.pubsub.common.Guid;

public class SCSROS2Participant
{
   private final Guid guid;
   private String name = "";

   public SCSROS2Participant(Guid guid)
   {
      this.guid = guid;
      this.name = guid.toString();
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   @Override
   public int hashCode()
   {
      return guid.hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      return guid.equals(obj);
   }

   public Guid getGuid()
   {
      return guid;
   }
}
