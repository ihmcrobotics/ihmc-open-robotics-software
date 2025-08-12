package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRContext;

import java.util.Set;

public class RDXStereoImagePanel
{
   private final RDX3DSituatedImagePanel leftPanel;
   private final RDX3DSituatedImagePanel rightPanel;
   private RDXVRModeManager vrModeManager;

   public RDXStereoImagePanel(RDXVRContext context, RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
      leftPanel = new RDX3DSituatedImagePanel(context, vrModeManager);
      rightPanel = new RDX3DSituatedImagePanel(context, vrModeManager);
   }

   /**
    * Update left and right panels for stereo viewing.
    *
    * @param leftImage    Left-eye image (Texture)
    * @param rightImage   Right-eye image (Texture)
    * @param leftCameraFrame  ReferenceFrame of the left camera
    * @param rightCameraFrame  ReferenceFrame of the right camera
    */
   public void update(Texture leftImage, Texture rightImage, ReferenceFrame leftCameraFrame, ReferenceFrame rightCameraFrame)
   {
      leftPanel.update(leftImage, leftCameraFrame);
      rightPanel.update(rightImage, rightCameraFrame);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT) && vrModeManager.getControls().getUseStereoVision().get())
      {
         leftPanel.getRenderables(renderables, pool, sceneLevels);
      }
      else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT) || sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         rightPanel.getRenderables(renderables, pool, sceneLevels);
      }
   }

   public void destroy()
   {
      leftPanel.destroy();
      rightPanel.destroy();
   }

   public RDX3DSituatedImagePanel getLeftPanel() { return leftPanel; }

   public RDX3DSituatedImagePanel getRightPanel() { return rightPanel; }
}
