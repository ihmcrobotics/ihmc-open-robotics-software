package us.ihmc.humanoidBehaviors.ui.tools;

import javafx.animation.AnimationTimer;

import java.util.function.LongConsumer;

/**
 * Allows to easily construct animation timers privately or to use multiple timers in a class.
 */
public class PrivateAnimationTimer extends AnimationTimer
{
   private final LongConsumer handler;

   public PrivateAnimationTimer(LongConsumer handler)
   {
      this.handler = handler;
   }

   @Override
   public void handle(long now)
   {
      handler.accept(now);
   }
}
