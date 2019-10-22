package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.messager.TopicListener;

public class MessagerCallback<T> implements TopicListener<T>
{
   private final TopicListener<T> listener;
   private volatile boolean enabled = true;

   public MessagerCallback(TopicListener<T> listener)
   {
      this.listener = listener;
   }

   @Override
   public void receivedMessageForTopic(T messageContent)
   {
      if (enabled)
      {
         listener.receivedMessageForTopic(messageContent);
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }
}
