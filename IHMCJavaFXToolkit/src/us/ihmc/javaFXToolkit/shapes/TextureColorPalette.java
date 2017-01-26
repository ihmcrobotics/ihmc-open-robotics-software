package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.image.Image;
import javafx.scene.paint.Color;

/**
 * {@code TextureColorPalette} defines a tool that maps texture coordinates to an image.
 * The main usage is for {@link JavaFXMultiColorMeshBuilder} that re-maps texture coordinates of the mesh vertices
 * to point to the image of the active {@link TextureColorPalette} and to allow the user to render a mesh with more 
 * than one color.
 * @author Sylvain Bertrand
 */
public interface TextureColorPalette
{
   /**
    * Retrieves the texture coordinates of a given {@link Color} in the image of this {@link TextureColorPalette}.
    * @param color the color to retrieve the texture coordinates of.
    * @return the corresponding texture coordinates.
    */
   public float[] getTextureLocation(Color color);

   /**
    * Retrieves the texture coordinates of a given {@code (red, green, blue)} in the image of this {@link TextureColorPalette}.
    * @param red the red component, in the range {@code 0.0-1.0}.
    * @param green the green component, in the range {@code 0.0-1.0}.
    * @param blue the blue component, in the range {@code 0.0-1.0}.
    * @return the corresponding texture coordinates.
    */
   public default float[] getTextureLocation(double red, double green, double blue)
   {
      return getTextureLocation((int) (255 * red), (int) (255 * green), (int) (255 * blue));
   }

   
   /**
    * Retrieves the texture coordinates of a given {@code (red, green, blue)} in the image of this {@link TextureColorPalette}.
    * @param red the red component, in the range {@code 0-255}.
    * @param green the green component, in the range {@code 0-255}.
    * @param blue the blue component, in the range {@code 0-255}.
    * @return the corresponding texture coordinates.
    */
   public default float[] getTextureLocation(int red, int green, int blue)
   {
      return getTextureLocation(Color.rgb(red, green, blue));
   }

   /**
    * @return the image to use with the texture coordinates computed.
    */
   public Image getColorPalette();
}