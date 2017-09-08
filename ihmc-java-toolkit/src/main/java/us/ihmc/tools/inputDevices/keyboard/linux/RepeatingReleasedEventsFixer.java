package us.ihmc.tools.inputDevices.keyboard.linux;

import java.awt.AWTEvent;
import java.awt.Component;
import java.awt.EventQueue;
import java.awt.Toolkit;
import java.awt.event.AWTEventListener;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.Timer;

/**
 * Copied verbatim from: <a href="http://tech.stolsvik.com/2010/05/linux-java-repeats-released-keyevents.html">here</a>.
 *
 * This fixes a bug in Linux where KEY_RELEASED events are fired while KEY_PRESSED is engaged, making handling of events that need to change on this switch difficult to manage.
 *
 * @author Endre Stolsvik
 *
 */
public class RepeatingReleasedEventsFixer implements AWTEventListener
{
   private final Map<Integer, ReleasedAction> _map = new LinkedHashMap<Integer, ReleasedAction>();

   public void install()
   {
      Toolkit.getDefaultToolkit().addAWTEventListener(this, AWTEvent.KEY_EVENT_MASK);
   }

   public void remove()
   {
      Toolkit.getDefaultToolkit().removeAWTEventListener(this);
   }

   @Override
   public void eventDispatched(AWTEvent event)
   {
      assert event instanceof KeyEvent : "Shall only listen to KeyEvents, so no other events shall come here";
      assert assertEDT(); // REMEMBER THAT THIS IS SINGLE THREADED, so no need for synch.

      // ?: Is this one of our synthetic RELEASED events?
      if (event instanceof Reposted)
      {
         // -> Yes, so we shalln't process it again.
         return;
      }

      // ?: KEY_TYPED event? (We're only interested in KEY_PRESSED and KEY_RELEASED).
      if (event.getID() == KeyEvent.KEY_TYPED)
      {
         // -> Yes, TYPED, don't process.
         return;
      }

      final KeyEvent keyEvent = (KeyEvent) event;

      // ?: Is this already consumed?
      // (Note how events are passed on to all AWTEventListeners even though a previous one consumed it)
      if (keyEvent.isConsumed())
      {
         return;
      }

      // ?: Is this RELEASED? (the problem we're trying to fix!)
      if (keyEvent.getID() == KeyEvent.KEY_RELEASED)
      {
         // -> Yes, so stick in wait

         /**
          * Really just wait until "immediately", as the point is that the subsequent PRESSED shall already have been
          * posted on the event queue, and shall thus be the direct next event no matter which events are posted
          * afterwards. The code with the ReleasedAction handles if the Timer thread actually fires the action due to
          * lags, by cancelling the action itself upon the PRESSED.
          */
         final Timer timer = new Timer(2, null);
         ReleasedAction action = new ReleasedAction(keyEvent, timer);
         timer.addActionListener(action);
         timer.start();

         _map.put(Integer.valueOf(keyEvent.getKeyCode()), action);

         // Consume the original
         keyEvent.consume();
      }
      else if (keyEvent.getID() == KeyEvent.KEY_PRESSED)
      {
         // Remember that this is single threaded (EDT), so we can't have races.
         ReleasedAction action = _map.remove(Integer.valueOf(keyEvent.getKeyCode()));

         // ?: Do we have a corresponding RELEASED waiting?
         if (action != null)
         {
            // -> Yes, so dump it
            action.cancel();
         }

         // System.out.println("PRESSED: [" + keyEvent + "]");
      }
      else
      {
         throw new AssertionError("All IDs should be covered.");
      }
   }

   /**
    * The ActionListener that posts the RELEASED {@link RepostedKeyEvent} if the {@link Timer} times out (and hence the
    * repeat-action was over).
    */
   private class ReleasedAction implements ActionListener
   {
      private final KeyEvent _originalKeyEvent;
      private Timer _timer;

      ReleasedAction(KeyEvent originalReleased, Timer timer)
      {
         _timer = timer;
         _originalKeyEvent = originalReleased;
      }

      void cancel()
      {
         assert assertEDT();
         _timer.stop();
         _timer = null;
         _map.remove(Integer.valueOf(_originalKeyEvent.getKeyCode()));
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         assert assertEDT();

         // ?: Are we already cancelled?
         // (Judging by Timer and TimerQueue code, we can theoretically be raced to be posted onto EDT by TimerQueue,
         // due to some lag, unfair scheduling)
         if (_timer == null)
         {
            // -> Yes, so don't post the new RELEASED event.
            return;
         }

         // Stop Timer and clean.
         cancel();

         // Creating new KeyEvent (we've consumed the original).
         KeyEvent newEvent = new RepostedKeyEvent((Component) _originalKeyEvent.getSource(), _originalKeyEvent.getID(), _originalKeyEvent.getWhen(), _originalKeyEvent.getModifiers(), _originalKeyEvent.getKeyCode(), _originalKeyEvent.getKeyChar(), _originalKeyEvent.getKeyLocation());

         // Posting to EventQueue.
         Toolkit.getDefaultToolkit().getSystemEventQueue().postEvent(newEvent);

         // System.out.println("Posted synthetic RELEASED [" + newEvent + "].");
      }
   }

   /**
    * Marker interface that denotes that the {@link KeyEvent} in question is reposted from some
    * {@link AWTEventListener}, including this. It denotes that the event shall not be "hack processed" by this class
    * again. (The problem is that it is not possible to state "inject this event from this point in the pipeline" - one
    * have to inject it to the event queue directly, thus it will come through this {@link AWTEventListener} too.
    */
   public interface Reposted
   {
      // marker
   }

   /**
    * Dead simple extension of {@link KeyEvent} that implements {@link Reposted}.
    */
   @SuppressWarnings("serial")
   public static class RepostedKeyEvent extends KeyEvent implements Reposted
   {
      public RepostedKeyEvent(Component source, int id, long when, int modifiers, int keyCode, char keyChar, int keyLocation)
      {
         super(source, id, when, modifiers, keyCode, keyChar, keyLocation);
      }
   }

   private static boolean assertEDT()
   {
      if (!EventQueue.isDispatchThread())
      {
         throw new AssertionError("Not EDT, but [" + Thread.currentThread() + "].");
      }

      return true;
   }
}
