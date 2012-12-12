package us.ihmc.graphics3DAdapter;

import java.awt.GraphicsDevice;
import java.net.URL;

import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.camera.CameraTrackAndDollyVariablesHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;
import us.ihmc.graphics3DAdapter.holders.NavigatingCameraHolder;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface Graphics3DAdapter
{
   public void addRootNode(Graphics3DNode rootNode);
   public void removeRootNode(Graphics3DNode rootNode);
   
   public ViewportAdapter createNewViewport(CameraTrackAndDollyVariablesHolder cameraTrackAndDollyYoVariablesHolder, GraphicsDevice graphicsDevice,
         NavigatingCameraHolder navigatingCameraHolder);
   
   public void setHeightMap(HeightMap heightMap);
   
   public Object getGraphicsConch();
   public void setGroundVisible(boolean isVisible);
   
   public RayCollisionAdapter getRayCollisionAdapter();
   
   public void addSelectedListener(SelectedListener selectedListener);
   
   public void destroy();
   
   public void setBackgroundColor(Color3f color);

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode);

   public void setGroundAppearance(YoAppearanceDefinition app);
}
