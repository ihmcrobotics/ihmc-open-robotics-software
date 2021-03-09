package us.ihmc.gdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Window;

import java.awt.*;
import java.util.function.Supplier;

public class GDXLinuxGUIRecorder //extends LinuxGUIRecorder
{
   public GDXLinuxGUIRecorder(int fps, double quality, String guiName)
   {
//      super(createWindowBoundsProvider(), fps, quality, guiName);
//   }

//   public static Supplier<Rectangle> createWindowBoundsProvider()
//   {
//      return () ->
//      {
//         Lwjgl3Graphics graphics = (Lwjgl3Graphics) Gdx.graphics;
//         Lwjgl3Window window = graphics.getWindow();
//         int x = window.getPositionX();
//         int y = window.getPositionY();
//         int width = graphics.getWidth();
//         int height = graphics.getHeight();
//
//         return new Rectangle(x, y, width, height);
//      };
   }
}
