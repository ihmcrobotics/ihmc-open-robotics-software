package us.ihmc.jMonkeyEngineToolkit;

import java.awt.GraphicsDevice;
import java.net.URL;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.graphicsDescription.input.mouse.MouseListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.lidar.LidarScanParameters;

public interface Graphics3DAdapter 
{
   public void setupSky();
   
   public void addRootNode(Graphics3DNode rootNode);

   public void removeRootNode(Graphics3DNode rootNode);

   public ViewportAdapter createNewViewport(GraphicsDevice graphicsDevice, boolean isMainViewport, boolean isOffScreen);

   public void closeViewport(ViewportAdapter viewport);

   public void setHeightMap(HeightMap heightMap);

   public Object getGraphicsConch();

   public void setGroundVisible(boolean isVisible);

   public void addSelectedListener(SelectedListener selectedListener);

   public void addKeyListener(KeyListener keyListener);

   public void addMouseListener(MouseListener mouseListener);
   
   public void addMouse3DListener(Mouse3DListener mouse3DListener);

   public void closeAndDispose();

   public void setBackgroundColor(MutableColor color);

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode);

   public void setGroundAppearance(AppearanceDefinition app);

   public void freezeFrame(Graphics3DNode rootJoint);

   public ContextManager getContextManager();

   public GPULidar createGPULidar(int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange);

   public GPULidar createGPULidar(GPULidarListener listener, int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange);
   
   public GPULidar createGPULidar(GPULidarListener listener, LidarScanParameters lidarScanParameters);
   
   public GPULidar createGPULidar(LidarScanParameters lidarScanParameters);
   
   public void play();

   public void pause();
}
