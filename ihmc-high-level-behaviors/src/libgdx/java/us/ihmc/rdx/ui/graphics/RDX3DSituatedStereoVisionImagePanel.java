package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.vr.RDXVRModeManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDX3DSituatedStereoVisionImagePanel
{
   private static final RobotSide DEFAULT_SIDE = RobotSide.LEFT;

   private final SideDependentList<RDX3DSituatedImagePanel> imagePanels = new SideDependentList<>();

   public RDX3DSituatedStereoVisionImagePanel(RDXVRContext context, RDXVRModeManager vrModeManager)
   {
      imagePanels.set(RobotSide.LEFT, new RDX3DSituatedImagePanel(context, vrModeManager));
      imagePanels.set(RobotSide.RIGHT, new RDX3DSituatedImagePanel(context, vrModeManager));
   }

   public void update(RobotSide side, Texture texture)
   {
      if (side != DEFAULT_SIDE)
      {
         imagePanels.get(side).changeFrameToPanel(imagePanels.get(DEFAULT_SIDE));
      }

      imagePanels.get(side).update(texture);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT))
      {
         imagePanels.get(RobotSide.LEFT).getRenderables(renderables, pool, sceneLevels);
      }
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         imagePanels.get(RobotSide.RIGHT).getRenderables(renderables, pool, sceneLevels);
      }
      else
      {
         imagePanels.get(DEFAULT_SIDE).getRenderables(renderables, pool, sceneLevels);
      }
   }
}
