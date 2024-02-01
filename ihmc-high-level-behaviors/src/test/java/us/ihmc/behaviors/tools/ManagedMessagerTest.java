package us.ihmc.behaviors.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.behaviors.tools.ManagedMessager;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.TopicListener;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

public class ManagedMessagerTest
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("TestRoot");
   private static final MessagerAPIFactory.CategoryTheme TestTheme = apiFactory.createCategoryTheme("Test");

   public final MessagerAPIFactory.Topic<String> TopicOne = topic("TopicOne");
   public final MessagerAPIFactory.Topic<String> TopicTwo = topic("TopicTwo");
   public final MessagerAPIFactory.Topic<String> TopicThree = topic("TopicThree");

   @Test
   public void test()
   {
      MessagerAPIFactory messagerAPIFactory = new MessagerAPIFactory();
      SharedMemoryMessager sharedMemoryMessager = new SharedMemoryMessager(create());
      ManagedMessager managedMessager = new ManagedMessager(sharedMemoryMessager);
      try
      {
         sharedMemoryMessager.startMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      TypedNotification<String> messageNotification = new TypedNotification<>();
      TopicListener<String> stringTopicListener = messageNotification::set;
      managedMessager.addTopicListener(TopicOne, stringTopicListener);

      managedMessager.submitMessage(TopicOne, "1");

      assertTrue(messageNotification.poll());
      assertEquals("1", messageNotification.read());

      managedMessager.setEnabled(false);

      managedMessager.submitMessage(TopicOne, "2");
      assertFalse(messageNotification.poll());

      managedMessager.setEnabled(true);

      managedMessager.submitMessage(TopicOne, "3");
      assertTrue(messageNotification.poll());
      assertEquals("3", messageNotification.read());

      managedMessager.removeTopicListener(TopicOne, stringTopicListener);

      managedMessager.submitMessage(TopicOne, "4");
      assertFalse(messageNotification.poll());

      AtomicReference<String> input = managedMessager.createInput(TopicTwo);

      managedMessager.submitMessage(TopicTwo, "5");
      assertEquals("5", input.get());

      managedMessager.setEnabled(false);

      managedMessager.submitMessage(TopicTwo, "6");
      assertEquals("5", input.get());

      managedMessager.removeInput(TopicTwo, input);
      managedMessager.setEnabled(true);

      managedMessager.submitMessage(TopicTwo, "7");
      assertEquals("5", input.get());

      managedMessager.setEnabled(false);

      AtomicReference<String> input2 = managedMessager.createInput(TopicThree);
      TypedNotification<String> messageNotification2 = new TypedNotification<>();
      TopicListener<String> stringTopicListener2 = messageNotification2::set;
      managedMessager.addTopicListener(TopicThree, stringTopicListener2);

      managedMessager.submitMessage(TopicThree, "8");

      assertFalse(messageNotification2.poll());
      assertNull(input2.get());

      managedMessager.setEnabled(true);

      managedMessager.submitMessage(TopicThree, "9");
      assertTrue(messageNotification2.poll());
      assertEquals("9", messageNotification2.read());
      assertEquals("9", input2.get());
   }

   private <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(TestTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
