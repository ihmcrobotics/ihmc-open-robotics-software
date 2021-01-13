package us.ihmc.gdx.application;

import com.badlogic.gdx.graphics.g3d.RenderableProvider;

public class GDXRenderable
{
   private final RenderableProvider renderableProvider;
   private final GDXSceneLevel sceneType;

   public GDXRenderable(RenderableProvider renderableProvider, GDXSceneLevel sceneType)
   {
      this.renderableProvider = renderableProvider;
      this.sceneType = sceneType;
   }

   public RenderableProvider getRenderableProvider()
   {
      return renderableProvider;
   }

   public GDXSceneLevel getSceneType()
   {
      return sceneType;
   }
}
