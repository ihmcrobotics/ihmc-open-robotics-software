package us.ihmc.jMonkeyEngineToolkit.jme.examples;

import java.net.URL;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.material.Material;
import com.jme3.system.AppSettings;
import com.jme3.texture.Texture;
import com.jme3.ui.Picture;

import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;

public class GPUMeshDistortionTest extends SimpleApplication
{

   public static void main(String[] args)
   {
      GPUMeshDistortionTest app = new GPUMeshDistortionTest();
      AppSettings settings = new AppSettings(true);
      settings.setResolution(1280, 720);
      app.setPauseOnLostFocus(false);
      app.setShowSettings(false);
      app.setSettings(settings);

      app.start(); // start the game

   }

   @Override
   public void simpleInitApp()
   {
      URL url = JMERenderer.class.getResource("JMEGraphics3DAdapterResources");
      assetManager.registerLocator(url.getPath(), FileLocator.class);
//      cam.setParallelProjection(true);
//      cam.setViewPort(0, 1f, 0f, 1f);
//      cam.setFrustum(1f, 2f, -0.5f, 0.5f, 0.5f, -0.5f);
//      cam.setLocation(new Vector3f());
//      cam.setRotation(new Quaternion());
//      cam.resize(1280, 720, true);
      
      Picture q = new Picture("Distored");
      Material mat = new Material(assetManager, "lidar/Distortion.j3md");
      Texture tex = assetManager.loadTexture("Interface/Logo/Monkey.jpg");
      mat.setTexture("tex1", tex);
      
      flyCam.setDragToRotate(true);
      q.setMaterial(mat);
      q.setLocalTranslation(0f, 0f, 0f);
      q.updateGeometricState();
      guiNode.attachChild(q);
   }
}
