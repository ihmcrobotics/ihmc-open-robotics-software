package us.ihmc.jMonkeyEngineToolkit.jme;

import java.awt.Color;
import java.awt.GraphicsDevice;
import java.net.URL;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DSpotLight;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.graphicsDescription.input.mouse.MouseListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.ContextManager;
import us.ihmc.jMonkeyEngineToolkit.GPULidarListener;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DBackgroundScaleMode;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer.RenderType;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.JMEGPULidar;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.lidar.LidarScanParameters;

/*
* Pass-through class to avoid having to import JME on all projects
 */
public class JMEGraphics3DAdapter implements Graphics3DAdapter
{
   private JMERenderer jmeRenderer = new JMERenderer(RenderType.AWTPANELS);

   public JMEGraphics3DAdapter()
   {
      this(true);
   }

   public JMEGraphics3DAdapter(boolean setupSky)
   {
      if (setupSky)
      {
         jmeRenderer.setupSky();
      }
   }

   public void setupSky()
   {
      jmeRenderer.setupSky();
   }

   public void addRootNode(Graphics3DNode rootNode)
   {
      jmeRenderer.addRootNode(rootNode);
   }

   public void removeRootNode(Graphics3DNode rootNode)
   {
      jmeRenderer.removeRootNode(rootNode);
   }

   public Object getGraphicsConch()
   {
      return jmeRenderer.getGraphicsConch();
   }

   public void setHeightMap(HeightMap heightMap)
   {
      jmeRenderer.setHeightMap(heightMap);
   }

   public void setGroundVisible(boolean isVisible)
   {
      jmeRenderer.setGroundVisible(isVisible);
   }

   public void addSelectedListener(SelectedListener selectedListener)
   {
      jmeRenderer.addSelectedListener(selectedListener);
   }

   public ViewportAdapter createNewViewport(GraphicsDevice graphicsDevice, boolean isMainViewport, boolean isOffScreen)
   {
      return jmeRenderer.createNewViewport(graphicsDevice, isMainViewport, isOffScreen);
   }

   public void closeAndDispose()
   {
      jmeRenderer.closeAndDispose();
      jmeRenderer = null;
   }

   public void setBackgroundColor(MutableColor color)
   {
      jmeRenderer.setBackgroundColor(color);
   }

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
      jmeRenderer.setBackgroundImage(fileURL, backgroundScaleMode);
   }

   public void setGroundAppearance(AppearanceDefinition app)
   {
      jmeRenderer.setGroundAppearance(app);
   }

   public void addKeyListener(KeyListener keyListener)
   {
      jmeRenderer.addKeyListener(keyListener);
   }

   public void addMouseListener(MouseListener mouseListener)
   {
      jmeRenderer.addMouseListener(mouseListener);
   }

   @Override
   public void addMouse3DListener(Mouse3DListener mouse3DListener)
   {
      jmeRenderer.addMouse3DListener(mouse3DListener);
   }

   public void freezeFrame(Graphics3DNode rootJoint)
   {
      jmeRenderer.freezeFrame(rootJoint);
   }

   public ContextManager getContextManager()
   {
      return jmeRenderer.getContextManager();
   }

   public void closeViewport(ViewportAdapter viewport)
   {
      if (jmeRenderer != null)
         jmeRenderer.closeViewport(viewport);
   }

   public JMERenderer getRenderer()
   {
      return jmeRenderer;
   }

   @Override
   public JMEGPULidar createGPULidar(int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      return jmeRenderer.createGPULidar(pointsPerSweep, scanHeight, fieldOfView, minRange, maxRange);
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      return jmeRenderer.createGPULidar(listener, pointsPerSweep, scanHeight, fieldOfView, minRange, maxRange);
   }

   @Override
   public JMEGPULidar createGPULidar(GPULidarListener listener, LidarScanParameters lidarScanParameters)
   {
      return jmeRenderer.createGPULidar(listener, lidarScanParameters);
   }

   @Override
   public JMEGPULidar createGPULidar(LidarScanParameters lidarScanParameters)
   {
      return jmeRenderer.createGPULidar(lidarScanParameters);
   }

   @Override
   public void play()
   {
      jmeRenderer.play();
   }

   @Override
   public void pause()
   {
      jmeRenderer.pause();

   }


   public void setAmbientLightBrightness(float brightness)
   {
      jmeRenderer.setAmbientLightBrightness(brightness);
   }

   @Override
   public void addDirectionalLight(Color color, Vector3D direction)
   {
      jmeRenderer.addDirectionalLight(color, direction);
   }

   @Override
   public void clearDirectionalLights()
   {
      jmeRenderer.clearDirectionalLights();
   }

   @Override
   public void setAmbientLight(Color color)
   {
      jmeRenderer.setAmbientLight(color);
   }

   @Override
   public void setupSky(String skyBox)
   {
      jmeRenderer.setupSky(skyBox);
   }

   @Override
   public void setupSky(String west, String east, String north, String south, String up, String down)
   {
      jmeRenderer.setupSky(west, east, north, south, up, down);

   }
   
   @Override
   public void addSpotLight(Graphics3DSpotLight spotLight)
   {
      jmeRenderer.addSpotLight(spotLight);
   }

   @Override
   public void removeSpotLight(Graphics3DSpotLight spotLight)
   {
      jmeRenderer.removeSpotLight(spotLight);
   }
}
