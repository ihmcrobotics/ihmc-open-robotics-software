package us.ihmc.simulationconstructionset.eventBased;

public class Event
{
   /**
    * Type of event
    */
   private final EventType type;

   /**
    * Comment associated with the event
    */
   private final String comment;

   /**
    * ctor
    *
    * @param type event type
    */
   public Event(EventType type)
   {
      this(type, "");
   }

   /**
    * ctor
    *
    * @param type event type
    * @param comment comment associated with the event
    */
   public Event(EventType type, String comment)
   {
      this.type = type;
      this.comment = comment;
   }

   public EventType getType()
   {
      return type;
   }

   public String getComment()
   {
      return comment;
   }

   @Override
   public String toString()
   {
      String ret = "Event" + "\n" + "Type: " + type + "\n";
      if (comment.length() > 0)
      {
         ret += "Comment: " + comment;
      }

      return ret;
   }
}
