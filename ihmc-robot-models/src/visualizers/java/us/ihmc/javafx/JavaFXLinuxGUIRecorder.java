package us.ihmc.javafx;

import javafx.stage.Stage;
import us.ihmc.gui.LinuxGUIRecorder;

import java.awt.*;
import java.util.function.Supplier;

public class JavaFXLinuxGUIRecorder extends LinuxGUIRecorder
{
   public JavaFXLinuxGUIRecorder(Stage primaryStage, int fps, double quality, String guiName)
   {
      super(createWindowBoundsProvider(primaryStage), fps, quality, guiName);
   }

   public static Supplier<Rectangle> createWindowBoundsProvider(Stage primaryStage)
   {
      return () ->
      {
         int x = (int) primaryStage.getX();
         int y = (int) primaryStage.getY();
         int width = (int) primaryStage.getWidth();
         int height = (int) primaryStage.getHeight();

         return new Rectangle(x, y, width, height);
      };
   }
}
