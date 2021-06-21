package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

public abstract class ExampleAbstractGDXBehaviorUIInterface extends GDXBehaviorUIInterface
{
   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {

   }

   @Override
   public void renderTreeNode()
   {

   }

   @Override
   public void renderInternal()
   {

   }

   @Override
   public void destroy()
   {

   }

   @Override
   public Point2D getTreeNodeInitialPosition()
   {
      return new Point2D();
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }
}
