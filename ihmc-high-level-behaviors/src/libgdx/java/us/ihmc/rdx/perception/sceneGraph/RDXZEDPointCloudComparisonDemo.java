package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.List;

public class RDXZEDPointCloudComparisonDemo
{
   private final ZEDColorDepthImageRetriever imageRetriever;
   private final OpenCLPointCloudExtractor pointCloudExtractor;

   private final RDXBaseUI baseUI;
   private final RDXPointCloudRenderer ihmcPointCloudRenderer;
   private final SwapReference<List<Point3D32>> ihmcPointCloudSwapReference = new SwapReference<>(new ArrayList<>(), new ArrayList<>());
   private final RDXPointCloudRenderer zedPointCloudRenderer;
   private final SwapReference<List<Point3D32>> zedPointCloudSwapReference = new SwapReference<>(new ArrayList<>(), new ArrayList<>());

   private volatile boolean done = false;

   public RDXZEDPointCloudComparisonDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0, ReferenceFrame::getWorldFrame, () -> true, () -> true);

      pointCloudExtractor = new OpenCLPointCloudExtractor();

      baseUI = new RDXBaseUI("Point Cloud Comparison");
      ihmcPointCloudRenderer = new RDXPointCloudRenderer();
      zedPointCloudRenderer = new RDXPointCloudRenderer();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      imageRetriever.start();
      while (!done)
         run();

      destroy();
   }

   private void run()
   {
      RawImage depthImage = imageRetriever.getLatestRawDepthImage();
      List<Point3D32> zedPointCloud = imageRetriever.getLatestPointCloud();
      synchronized (zedPointCloudSwapReference)
      {
         zedPointCloudSwapReference.getForThreadOne().clear();
         zedPointCloudSwapReference.getForThreadOne().addAll(zedPointCloud);
         zedPointCloudSwapReference.swap();
      }

      List<Point3D32> ihmcPointCloud = pointCloudExtractor.extractPointCloud(depthImage);
      synchronized (ihmcPointCloudSwapReference)
      {
         ihmcPointCloudSwapReference.getForThreadOne().clear();
         ihmcPointCloudSwapReference.getForThreadOne().addAll(ihmcPointCloud);
         ihmcPointCloudSwapReference.swap();
      }

      depthImage.release();
   }

   private void destroy()
   {
      imageRetriever.destroy();
      pointCloudExtractor.destroy();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            ihmcPointCloudRenderer.create(1280 * 720);
            zedPointCloudRenderer.create(1280 * 720);

            baseUI.getPrimaryScene().addRenderableProvider(ihmcPointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(zedPointCloudRenderer);
            baseUI.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            synchronized (zedPointCloudSwapReference)
            {
               zedPointCloudRenderer.setPointsToRender(zedPointCloudSwapReference.getForThreadTwo(), Color.RED);
               zedPointCloudRenderer.updateMesh();
            }

            synchronized (ihmcPointCloudSwapReference)
            {
               ihmcPointCloudRenderer.setPointsToRender(ihmcPointCloudSwapReference.getForThreadTwo(), Color.BLUE);
               ihmcPointCloudRenderer.updateMesh();
            }

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            done = true;
            ihmcPointCloudRenderer.dispose();
            zedPointCloudRenderer.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXZEDPointCloudComparisonDemo();
   }
}
