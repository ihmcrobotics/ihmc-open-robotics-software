package us.ihmc.gdx.input;

import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.SnapshotArray;

public class GDXInputMultiplexer implements InputProcessor
{
   private SnapshotArray<GDXInputAdapter> processors = new SnapshotArray(4);

   public GDXInputMultiplexer()
   {
   }

   public GDXInputMultiplexer(GDXInputAdapter... processors)
   {
      this.processors.addAll(processors);
   }

   public void addProcessor(int index, GDXInputAdapter processor)
   {
      if (processor == null)
         throw new NullPointerException("processor cannot be null");
      processors.insert(index, processor);
   }

   public void removeProcessor(int index)
   {
      processors.removeIndex(index);
   }

   public void addProcessor(GDXInputAdapter processor)
   {
      if (processor == null)
         throw new NullPointerException("processor cannot be null");
      processors.add(processor);
   }

   public void removeProcessor(GDXInputAdapter processor)
   {
      processors.removeValue(processor, true);
   }

   /**
    * @return the number of processors in this multiplexer
    */
   public int size()
   {
      return processors.size;
   }

   public void clear()
   {
      processors.clear();
   }

   public void setProcessors(GDXInputAdapter... processors)
   {
      this.processors.clear();
      this.processors.addAll(processors);
   }

   public void setProcessors(Array<GDXInputAdapter> processors)
   {
      this.processors.clear();
      this.processors.addAll(processors);
   }

   public SnapshotArray<GDXInputAdapter> getProcessors()
   {
      return processors;
   }

   public boolean keyDown(int keycode)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().keyDown(keycode))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean keyUp(int keycode)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().keyUp(keycode))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean keyTyped(char character)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().keyTyped(character))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().touchDown(screenX, screenY, pointer, button))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().touchUp(screenX, screenY, pointer, button))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean touchDragged(int screenX, int screenY, int pointer)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().touchDragged(screenX, screenY, pointer))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean touchDraggedDelta(int screenX, int screenY)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).touchDraggedDelta(screenX, screenY))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean mouseMoved(int screenX, int screenY)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().mouseMoved(screenX, screenY))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }

   public boolean scrolled(float amountX, float amountY)
   {
      Object[] items = processors.begin();
      try
      {
         for (int i = 0, n = processors.size; i < n; i++)
            if (((GDXInputAdapter) items[i]).getInputProcessor().scrolled(amountX, amountY))
               return true;
      }
      finally
      {
         processors.end();
      }
      return false;
   }
}
