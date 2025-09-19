package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   public void update(Mat leftImage,
                      PixelFormat leftPixelFormat,
                      Mat rightImage,
                      PixelFormat rightPixelFormat,
                      ReferenceFrame leftCameraFrame,
                      ReferenceFrame rightCameraFrame,
                      float verticalFOV)
   {
      panels.get(RobotSide.LEFT).update(leftImage, leftPixelFormat, leftCameraFrame, verticalFOV);
      panels.get(RobotSide.RIGHT).update(rightImage, rightPixelFormat, rightCameraFrame, verticalFOV);
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
