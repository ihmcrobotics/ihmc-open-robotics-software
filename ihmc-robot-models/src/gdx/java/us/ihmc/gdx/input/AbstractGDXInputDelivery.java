package us.ihmc.gdx.input;

import java.util.ArrayList;
import java.util.function.IntConsumer;

public abstract class AbstractGDXInputDelivery
{
   private ArrayList<IntConsumer> keyDownListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> keyUpListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> keyTypedListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseClickedListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseDownListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseUpListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseDraggedDeltaListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseMovedListeners = new ArrayList<>(1);
   private ArrayList<IntConsumer> mouseScrolledListeners = new ArrayList<>(1);

   public abstract boolean isButtonPressed(int mouseButton);

   public abstract boolean isKeyPressed(int key);

   public void addKeyDownListener(IntConsumer onKeyDown)
   {
      keyDownListeners.add(onKeyDown);
   }

   public void addKeyUpListener(IntConsumer onKeyUp)
   {
      keyUpListeners.add(onKeyUp);
   }

   public void addKeyTypedListener(IntConsumer onKeyTyped)
   {
      keyTypedListeners.add(onKeyTyped);
   }

   public void addMouseClickedListener(IntConsumer onMouseClicked)
   {
      mouseClickedListeners.add(onMouseClicked);
   }

   public void addMouseDownListener(IntConsumer onMouseDown)
   {
      mouseDownListeners.add(onMouseDown);
   }

   public void addMouseUpListener(IntConsumer onMouseUp)
   {
      mouseUpListeners.add(onMouseUp);
   }

   public void addMouseDraggedDeltaListener(IntConsumer onMouseDraggedDelta)
   {
      mouseDraggedDeltaListeners.add(onMouseDraggedDelta);
   }

   public void addMouseMovedListener(IntConsumer onMouseMoved)
   {
      mouseMovedListeners.add(onMouseMoved);
   }

   public void addMouseScrolledListener(IntConsumer onMouseScrolled)
   {
      mouseScrolledListeners.add(onMouseScrolled);
   }
}
