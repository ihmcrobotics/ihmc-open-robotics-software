package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;

import java.util.ArrayList;

public class GDX2DSprite implements GDX2DSpriteDrawable
{
   private final Sprite sprite;
   private final Texture texture;
   private double orientation;
   private boolean visible = true;
   private boolean flipX = false;
   private boolean flipY = false;

   public GDX2DSprite(String imageName)
   {
      this(new Texture(Gdx.files.internal(imageName), Pixmap.Format.RGBA8888, false));
   }

   public GDX2DSprite(Pixmap pixmap)
   {
      this(new Texture(pixmap));
   }

   public GDX2DSprite(Texture texture)
   {
      this.texture = texture;
      sprite = new Sprite(texture);
      orientation = 0.0;
   }

   @Override
   public void draw(SpriteBatch spriteBatch)
   {
      float x = sprite.getX();
      float y = sprite.getY();
      float originX = sprite.getWidth() / 2.0f;
      float originY = sprite.getHeight() / 2.0f;
      float width = sprite.getWidth();
      float height = sprite.getHeight();
      float scaleX = sprite.getScaleX();
      float scaleY = sprite.getScaleY();
      float rotation = (float) Math.toDegrees(orientation);
      int srcX = 0;
      int srcY = 0;
      int srcWidth = texture.getWidth();
      int srcHeight = texture.getHeight();
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
      sprite.setCenterX((float) x);
   }

   public void setY(double y)
   {
      sprite.setCenterY((float) y);
   }

   public void setHeightPreserveScale(double height)
   {
      int textureHeight = texture.getHeight();
      int textureWidth = texture.getWidth();
      float heightToWidthRatio = textureHeight / (float) textureWidth;
      float newWidth = (float) height / heightToWidthRatio;
      float newHeight = (float) height;
      sprite.setSize(newWidth, newHeight);
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public void getSpriteRenderables(ArrayList<GDX2DSpriteDrawable> sprites)
   {
      if (visible)
      {
         sprites.add(this);
      }
   }

   public double getPositionX()
   {
      return sprite.getX() + sprite.getWidth() / 2.0;

   }
   public double getPositionY()
   {
      return sprite.getY() + sprite.getHeight() / 2.0;
   }

   public void setOrientation(double orientation)
   {
      this.orientation = orientation;
   }

   public double getOrientation()
   {
      return orientation;
   }

   public Texture getTexture()
   {
      return texture;
   }

   public void hide()
   {
      this.visible = false;
   }

   public void show()
   {
      this.visible = true;
   }

   public void setVisible(boolean visible)
   {
      this.visible = visible;
   }

   public boolean isVisible()
   {
      return visible;
   }

   public boolean getFlipX()
   {
      return flipX;
   }

   public void setFlipX(boolean flipX)
   {
      this.flipX = flipX;
   }

   public boolean getFlipY()
   {
      return flipY;
   }

   public void setFlipY(boolean flipY)
   {
      this.flipY = flipY;
   }
}
