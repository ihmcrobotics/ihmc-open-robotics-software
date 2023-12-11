package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.colorVision.DualBlackflyUDPReceiver;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXDualBlackflySphericalProjection
{
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final ImDouble pupillaryDistance = new ImDouble(0.180724);
   private final RigidBodyTransform leftEyePose = new RigidBodyTransform();
   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImBoolean leftParametersVisible = new ImBoolean(true);
   private ImBoolean rightParametersVisible = new ImBoolean(true);

   public RDXDualBlackflySphericalProjection(RDXBaseUI baseUI)
   {
      baseUI.getImGuiPanelManager().addPanel("Projection", () ->
      {
         if (ImGuiTools.sliderDouble(labels.get("Pupillary distance"), pupillaryDistance, -15, 15))
         {

         }
         ImGui.separator();
         ImGuiTools.textBold("Left:");
         ImGui.indent();
         projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
         ImGui.unindent();
         ImGui.separator();
         ImGuiTools.textBold("Right:");
         ImGui.indent();
         projectionSpheres.get(RobotSide.RIGHT).renderImGuiWidgets();
         ImGui.unindent();
         ImGui.separator();
      });
   }

   public void create() throws Exception
   {
      projectionSpheres.get(RobotSide.LEFT).create();
      projectionSpheres.get(RobotSide.RIGHT).create();
      dualBlackflyUDPReceiver.start();
   }

   public void shutdown()
   {
      dualBlackflyUDPReceiver.stop();
   }

   public void render()
   {
      for (RobotSide side : RobotSide.values)
      {
         byte[] imageData = dualBlackflyUDPReceiver.getImageBuffers().get(side);

         if (imageData != null)
         {
            ImageDimensions imageDimensions = dualBlackflyUDPReceiver.getImageDimensions().get(side);
            BytePointer imageDataPointer = new BytePointer(imageData);

            Mat mat = new Mat(imageDimensions.getImageHeight(), imageDimensions.getImageWidth(), opencv_core.CV_8UC1);
            mat.data(imageDataPointer);

            Pixmap pixmap = new Pixmap(mat.cols(), mat.rows(), Pixmap.Format.RGBA8888);
            BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
            Mat rgba8Mat = new Mat(mat.rows(), mat.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
            opencv_imgproc.cvtColor(mat, rgba8Mat, opencv_imgproc.COLOR_BayerBG2RGBA);
            Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

            projectionSpheres.get(side).updateTexture(texture);

            rgba8Mat.close();
            pixmap.dispose();
            mat.close();
            imageDataPointer.close();
         }
      }

      leftEyePose.getTranslation().setY(pupillaryDistance.get());
      LibGDXTools.toLibGDX(leftEyePose, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
      else
      {
         projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
      }
   }
}
