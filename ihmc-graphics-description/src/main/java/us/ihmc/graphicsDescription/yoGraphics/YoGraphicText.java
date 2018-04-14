package us.ihmc.graphicsDescription.yoGraphics;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class YoGraphicText extends YoGraphicAbstractShape
{
   private final Graphics3DObject graphics3dObject;
   private final Font font = new Font("Lucida Sans", Font.BOLD, 26);
   private final String text;
   private final Color backgroundColor;
   private final Color textAppearance;

   public YoGraphicText(String name, String text, YoFramePoint3D framePoint, YoFrameYawPitchRoll frameOrientation, double scale, Color backgroundColor,
                        Color textAppearance)
   {
      super(name, framePoint, frameOrientation, scale);
      this.text = text;
      this.backgroundColor = backgroundColor;
      this.textAppearance = textAppearance;
      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);

      //initial guess for block and image dimensions
      double blockWidth = text.length() * .5;
      double blockHeight = 0.5;
      double blockThickness = 0.01;

      double imageScaleFactor = 50;
      int imageWidth = (int) (blockWidth * imageScaleFactor);
      int imageHeight = (int) (blockHeight * imageScaleFactor);

      BufferedImage appearance = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_4BYTE_ABGR);
      Graphics2D g = appearance.createGraphics();

      //text dimensions
      Rectangle2D bounds = font.getStringBounds(text, g.getFontRenderContext());
      g.dispose();
      int textHeight = (int) bounds.getHeight();
      int textWidth = (int) bounds.getWidth();

      //Adjust image and block dims to just fit text
      imageWidth = textWidth + 10;
      imageHeight = textHeight + 10;
      blockWidth = blockHeight / imageHeight * imageWidth;

      //Create new appearance and graphics context
      appearance = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_4BYTE_ABGR);
      g = appearance.createGraphics();

      //draw text (and background)
      g.setColor(backgroundColor);
      g.fillRect(0, 0, imageWidth, imageHeight);

      int x = (imageWidth - textWidth) / 2;
      int y = (imageHeight - textHeight) / 2 + textHeight;
      g.setColor(textAppearance);
      g.translate(x, y);
      g.setFont(font);
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g.drawString(text, 0, 0);

      g.dispose();

      graphics3dObject.addCube(blockWidth, blockHeight, blockThickness, new YoAppearanceTexture(appearance));
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   @Override
   public YoGraphic duplicate(YoVariableRegistry newRegistry)
   {
      return new YoGraphicText(getName(), text, yoFramePoint.duplicate(newRegistry), yoFrameOrientation.duplicate(newRegistry), scale, backgroundColor,
                               textAppearance);
   }
}
