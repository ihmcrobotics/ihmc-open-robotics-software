package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;

import java.util.ArrayList;

public class GDX2DSceneBasics
{
   private final ArrayList<GDX2DSprite> sprites = new ArrayList<>();

   private SpriteBatch spriteBatch;

   public void create()
   {
      spriteBatch = new SpriteBatch();
   }

   public void preRender(Camera camera)
   {
      spriteBatch.begin();
   }

   public void render()
   {
      for (GDX2DSprite sprite : sprites)
      {
         sprite.draw(spriteBatch);
      }
   }

   public void postRender(Camera camera, GDXSceneLevel sceneLevel)
   {
      spriteBatch.end();
   }

   public void dispose()
   {

   }

   public ArrayList<GDX2DSprite> getSprites()
   {
      return sprites;
   }
}
