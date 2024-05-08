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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator.CameraOrientation;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;
import java.util.function.BiConsumer;

public class RDXDualBlackflySphericalProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionHemisphere> projectionSpheres = new SideDependentList<>(RDXProjectionHemisphere::new);
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

            projectionSpheres.get(RobotSide.LEFT).updateMeshLazy();
            projectionSpheres.get(RobotSide.RIGHT).updateMeshLazy();

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

            for (RobotSide side : RobotSide.values)
            {
               RDXProjectionHemisphere projectionHemisphere = (RDXProjectionHemisphere) projectionSpheres.get(side);
               BiConsumer<Point3DReadOnly[], TexCoord2f[]> textureCoordinateCalculator = projectionHemisphere.getTextureCoordinateCalculator();

               if (!(textureCoordinateCalculator instanceof FisheyeTextureCalculator))
               {
                  BlackflyLensProperties properties = side == RobotSide.LEFT ?
                        BlackflyLensProperties.BFLY_U3_23S6C_FE185C086HA_1_17403057_MSA_CALIBRATED :
                        BlackflyLensProperties.BFLY_U3_23S6C_FE185C086HA_1_17372478_MSA_CALIBRATED;
                  textureCoordinateCalculator = new FisheyeTextureCalculator(CameraOrientation.X_DEPTH_POSITIVE,
                                                                             properties.getFocalLengthXForUndistortion(),
                                                                             properties.getFocalLengthYForUndistortion(),
                                                                             properties.getPrincipalPointXForUndistortion(),
                                                                             properties.getPrincipalPointYForUndistortion(),
                                                                             properties.getK1ForUndistortion(),
                                                                             properties.getK2ForUndistortion(),
                                                                             properties.getK3ForUndistortion(),
                                                                             properties.getK4ForUndistortion(),
                                                                             Math.toRadians(185.0));
                  projectionHemisphere.setTextureCoordinateCalculator(textureCoordinateCalculator);
               }
            }
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

                  projectionSpheres.get(side).updateMeshLazy();
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
