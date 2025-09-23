package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.zed.ZEDModelData;

import java.util.Set;

public class RDXStereoImagePanel
{
   private final SideDependentList<RDX3DSituatedImagePanel> panels = new SideDependentList<>();
   private final RDXVRModeControls vrControls;

   public RDXStereoImagePanel(RDXVRContext context, RDXVRModeControls vrControls)
   {
      this.vrControls = vrControls;
      for (RobotSide side : RobotSide.values)
      {
         panels.put(side, new RDX3DSituatedImagePanel(context, vrControls, side == RobotSide.LEFT));
      }
   }

   public void update(Texture leftImage, Texture rightImage, ReferenceFrame leftCameraFrame, ReferenceFrame rightCameraFrame, float verticalFOV)
   {
      panels.get(RobotSide.LEFT).update(leftImage, leftCameraFrame, verticalFOV);
      panels.get(RobotSide.RIGHT).update(rightImage, rightCameraFrame, verticalFOV);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT) && vrControls.getUseStereoVision().get())
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

   public boolean isOccludingView(Point3DReadOnly point)
   {
      return panels.get(RobotSide.LEFT).isOccludingView(point);
   }
}
