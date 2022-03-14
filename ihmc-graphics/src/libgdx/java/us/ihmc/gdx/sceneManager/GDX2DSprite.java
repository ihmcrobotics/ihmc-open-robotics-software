package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;

public class GDX2DSprite
{
   Sprite sprite;
   double orientation;
   Texture texture;
   float scale = 1.0f;

   public GDX2DSprite(String imageName)
   {
      texture = new Texture(imageName);
      sprite = new Sprite(texture);
      orientation = 0.0;
   }

   public void draw(SpriteBatch spriteBatch)
   {
      float x = (float) sprite.getX();
      float y = (float) sprite.getY();
      float originX = 0.0f;
      float originY = 0.0f;
      float width = (float) sprite.getWidth();
      float height = (float) sprite.getHeight();
      float scaleX = scale;
      float scaleY = scale;
      float rotation = (float) orientation;
      int srcX = 0;
      int srcY = 0;
      int srcWidth = (int) sprite.getWidth(); // TODO: Check
      int srcHeight = (int) sprite.getHeight(); // TODO: Check
      boolean flipX = false;
      boolean flipY = false;
      spriteBatch.draw(sprite.getTexture(),
                       x,
                       y,
                       originX,
                       originY,
                       width,
                       height,
                       scaleX,
                       scaleY,
                       rotation,
                       srcX,
                       srcY,
                       srcWidth,
                       srcHeight,
                       flipX,
                       flipY);
   }

   public void setX(double x)
   {
      sprite.setX((float) x);
   }

   public void setY(double y)
   {
      sprite.setY((float) y);
   }

   public void setScale(float scale)
   {
      this.scale = scale;
   }

   public float getScale()
   {
      return scale;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public double getOrientation()
   {
      return orientation;
   }

   public Texture getTexture()
   {
      return texture;
   }
}
