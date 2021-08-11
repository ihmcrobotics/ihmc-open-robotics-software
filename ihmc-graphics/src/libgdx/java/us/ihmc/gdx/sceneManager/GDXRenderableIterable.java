package us.ihmc.gdx.sceneManager;

import java.util.HashSet;
import java.util.Iterator;
import java.util.NoSuchElementException;

class GDXRenderableIterable implements Iterable<GDXRenderable>
{
   private final HashSet<GDXRenderable> renderables;
   private final GDXSceneLevel level;

   protected GDXRenderableIterable(HashSet<GDXRenderable> renderables)
   {
      this(renderables, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   protected GDXRenderableIterable(HashSet<GDXRenderable> renderables, GDXSceneLevel level)
   {
      this.renderables = renderables;
      this.level = level;
   }

   @Override
   public Iterator<GDXRenderable> iterator()
   {
      return new GDXRenderableIterator();
   }

   private class GDXRenderableIterator implements Iterator<GDXRenderable>
   {
      private final HashSet<GDXRenderable> renderablesInternal;

      private GDXRenderableIterator()
      {
         renderablesInternal = new HashSet<>(renderables);
      }

      @Override
      public boolean hasNext()
      {
         for (GDXRenderable renderable : renderablesInternal)
         {
            if (renderable.getSceneType() == level)
            {
               return true;
            }
         }

         return false;
      }

      @Override
      public GDXRenderable next()
      {
         Iterator<GDXRenderable> it = renderablesInternal.iterator();
         while (it.hasNext())
         {
            GDXRenderable renderable = it.next();
            it.remove();

            if (renderable.getSceneType() == level)
            {
               return renderable;
            }
         }

         throw new NoSuchElementException();
      }
   }
}
