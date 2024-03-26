package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPReceiver;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXDualBlackflySphericalProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final ImDouble pupillaryDistance = new ImDouble(0.67);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodyTransform leftEyePose = new RigidBodyTransform();

//   private final DualBlackflyReader dualBlackflyReader = new DualBlackflyReader();
   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   public RDXDualBlackflySphericalProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            try
            {
//               dualBlackflyReader.start();
               dualBlackflyUDPReceiver.start();
            }
            catch (Exception e)
            {
               e.printStackTrace();
               baseUI.dispose();
               return;
            }

            baseUI.create();

            projectionSpheres.get(RobotSide.LEFT).create();
            projectionSpheres.get(RobotSide.RIGHT).create();

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
               if (ImGuiTools.sliderDouble(labels.get("Pupillary distance"), pupillaryDistance, -15, 15))
               {

               }
               ImGui.text("Left:");
               projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
               ImGui.text("Right:");
               projectionSpheres.get(RobotSide.RIGHT).renderImGuiWidgets();
            });
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
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

         @Override
         public void render()
         {
            for (RobotSide side : RobotSide.values)
            {
//               DualBlackflyReader.SpinnakerBlackflyReaderThread readerThread = dualBlackflyReader.getSpinnakerBlackflyReaderThreads().get(side);
               byte[] imageBuffer = dualBlackflyUDPReceiver.getImageBuffers().get(side);

//               AtomicReference<BytePointer> latestImage = readerThread.getLatestImageDataPointer();
               if (imageBuffer != null)
               {
                  ImageDimensions imageDimensions = dualBlackflyUDPReceiver.getImageDimensions().get(side);

                  BytePointer latestImageData = new BytePointer(imageBuffer);

                  Mat mat = new Mat(imageDimensions.getImageHeight(), imageDimensions.getImageWidth(), opencv_core.CV_8UC1);
                  mat.data(latestImageData);

                  Pixmap pixmap = new Pixmap(mat.cols(), mat.rows(), Pixmap.Format.RGBA8888);
                  BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
                  Mat rgba8Mat = new Mat(mat.rows(), mat.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
                  opencv_imgproc.cvtColor(mat, rgba8Mat, opencv_imgproc.COLOR_BayerBG2RGBA);
                  Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

                  projectionSpheres.get(side).updateTexture(texture, 1.0f);

                  rgba8Mat.close();
                  pixmap.dispose();
                  mat.close();
                  latestImageData.close();
               }
            }

            leftEyePose.getTranslation().setY(pupillaryDistance.get());
            LibGDXTools.toLibGDX(leftEyePose, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
//            dualBlackflyReader.stop();
            dualBlackflyUDPReceiver.stop();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      Mat mat = new Mat(); // Get opencv loaded before things start
      new RDXDualBlackflySphericalProjectionDemo();
   }
}
