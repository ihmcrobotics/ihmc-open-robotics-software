package us.ihmc.quadrupedUI.skybox;

import org.fxyz3d.scene.Skybox;

import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SubScene;
import javafx.scene.image.Image;
import javafx.scene.transform.Rotate;

public class QuadrupedSkybox3D
{
   private final Skybox skybox;

   public QuadrupedSkybox3D(SubScene subScene)
   {
      this(100000.0, subScene);
   }

   public QuadrupedSkybox3D(double size, SubScene subScene)
   {
      PerspectiveCamera camera = (PerspectiveCamera) subScene.getCamera();
      skybox = loadCloudyCrown(size, camera);
      skybox.getTransforms().add(new Rotate(-90.0, Rotate.X_AXIS));
   }

   public static Skybox loadCloudyCrown(double size, PerspectiveCamera camera)
   {
      return loadSixImageSkybox("cloudyCrown/v02_Evening", "png", size, camera);
   }

   public static Skybox loadCartoonLandscape(double size, PerspectiveCamera camera)
   {
      Image image = new Image(QuadrupedSkybox3D.class.getResourceAsStream("skybox-cartoon.png"));
      return new Skybox(image, size, camera);
   }

   private static Skybox loadSixImageSkybox(String directoryPath, String fileExtension, double size, PerspectiveCamera camera)
   {
      Image topImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Up." + fileExtension));
      Image bottomImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Down." + fileExtension));
      Image leftImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Left." + fileExtension));
      Image rightImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Right." + fileExtension));
      Image frontImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Front." + fileExtension));
      Image backImg = new Image(QuadrupedSkybox3D.class.getResourceAsStream(directoryPath + "/Back." + fileExtension));
      return new Skybox(topImg, bottomImg, leftImg, rightImg, frontImg, backImg, size, camera);
   }

   public Node getSkybox()
   {
      return skybox;
   }
}
