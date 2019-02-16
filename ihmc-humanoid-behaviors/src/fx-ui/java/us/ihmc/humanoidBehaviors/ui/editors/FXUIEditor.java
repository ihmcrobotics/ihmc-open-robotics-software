package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.animation.AnimationTimer;
import javafx.event.Event;

/**
 * An editor such that:
 *
 * - each kind of editor is instantiated only once in an application
 * - only one editor can be active at a time
 * - and editor receives user input, broadcasting any state needed for it's graphics
 */
public abstract class FXUIEditor extends AnimationTimer
{


   public static final FXUIEditor NONE = new FXUIEditor() {
      @Override
      public void handle(long now)
      {

      }
   };
}
