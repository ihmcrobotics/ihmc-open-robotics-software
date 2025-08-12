package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXStereoImagePanel
{
   private final SideDependentList<RDX3DSituatedImagePanel> panels = new SideDependentList<>();
   private RDXVRModeManager vrModeManager;

   public RDXStereoImagePanel(RDXVRContext context, RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
      for (RobotSide side : RobotSide.values)
      {
         panels.put(side, new RDX3DSituatedImagePanel(context, vrModeManager));
      }
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
      panels.get(RobotSide.LEFT).update(leftImage, leftCameraFrame);
      panels.get(RobotSide.RIGHT).update(rightImage, rightCameraFrame);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT) && vrModeManager.getControls().getUseStereoVision().get())
      {
         panels.get(RobotSide.RIGHT).getRenderables(renderables, pool, sceneLevels);
      }
      else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT) || sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         panels.get(RobotSide.LEFT).getRenderables(renderables, pool, sceneLevels);
      }
   }

   public void destroy()
   {
      for (RobotSide side : RobotSide.values)
      {
         panels.get(side).destroy();
      }
   }

   public RDX3DSituatedImagePanel getPanel(RobotSide side) { return panels.get(side); }
}
