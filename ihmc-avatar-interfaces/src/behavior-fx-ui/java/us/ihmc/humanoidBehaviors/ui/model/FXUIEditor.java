package us.ihmc.humanoidBehaviors.ui.model;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.SubScene;
import javafx.scene.input.MouseEvent;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.references.ActivationReference;
import us.ihmc.messager.Messager;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An editor such that:
 *
 * - each kind of editor is instantiated only once in an application
 * - only one editor can be active at a time
 * - and editor receives user input, broadcasting any state needed for it's graphics
 */
public abstract class FXUIEditor extends AnimationTimer
{
   protected final Messager messager;
   protected final SubScene subScene;

   protected final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   protected final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;

   protected final ActivationReference<FXUIEditor> activeEditor;
   protected final AtomicReference<FXUIStateMachine> activeStateMachine;

   public FXUIEditor(Messager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;

      activeEditor = new ActivationReference<>(messager.createInput(BehaviorUI.API.ActiveEditor, null), this);
      activeStateMachine = messager.createInput(BehaviorUI.API.ActiveStateMachine, null);
   }

   protected void mouseMoved(MouseEvent event)
   {

   }

   protected void mouseClicked(MouseEvent event)
   {

   }
}
