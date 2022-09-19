package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.logging.PerceptionDataLoader;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXMocapVisualizer;
import us.ihmc.gdx.ui.graphics.live.GDXOpenCVVideoVisualizer;
import us.ihmc.gdx.ui.graphics.live.LogVideoLoader;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.UnitConversions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;

public class GDXSCS2LogSession extends GDXSCS2Session
{
   private LogSession session;

   private LogVideoLoader logVideoLoader;
   private GDXMocapVisualizer mocapVisualizer;
   private PerceptionDataLoader loader;

   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(200000, Point3D32::new);
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final ArrayList<GDXOpenCVVideoVisualizer> imagePanels = new ArrayList<>();

   private int cloudIndex = 0;

   public GDXSCS2LogSession(LogSession session)
   {
      super(session);
      this.session = session;
      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));

      imagePanels.add(new GDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
      imagePanels.add(new GDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaSouth", false));
   }

   public void createLogVideoLoader(String logVideoFilePath, String logVideoTimestampFilePath) throws IOException
   {
      logVideoLoader = new LogVideoLoader(logVideoFilePath, logVideoTimestampFilePath);
   }

   public void createPerceptionDataLoader(String perceptionLogFile)
   {
      loader = new PerceptionDataLoader(perceptionLogFile);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      super.create(baseUI);

      baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
      pointCloudRenderer.create(2048 * 64, 1);

      for (GDXOpenCVVideoVisualizer visualizer : imagePanels)
      {
         baseUI.getPrimaryScene().addRenderableProvider(visualizer);
      }
   }

   public void update()
   {
      super.update();

      if (session.getActiveMode() == SessionMode.RUNNING)
      {
         if (cloudIndex % 100 == 0)
         {
            loader.loadPointCloud("/os_cloud_node/points", cloudIndex / 100, points);
            pointCloudRenderer.setPointsToRender(points);
            if (!points.isEmpty())
            {
               pointCloudRenderer.updateMesh();
            }
         }

         Mat mat = logVideoLoader.loadNextFrameAsOpenCVMat(session.getLogDataReader().getTimestamp().getValue());

         BytePointer jpegImageBytePointer = new BytePointer();
         IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

         opencv_imgcodecs.imencode(".jpg", mat, jpegImageBytePointer, compressionParameters);

         if (mat != null)
         {
            imagePanels.get(0).setImage(mat);
         }
      }

      cloudIndex++;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<GDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);
   }

   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPartOne();
      renderImGuiWidgetsPartTwo();

      for (GDXOpenCVVideoVisualizer visualizer : imagePanels)
      {
         visualizer.renderImGuiWidgets();
      }
   }

   protected void renderImGuiWidgetsPartOne()
   {
      super.renderImGuiWidgetsPartOne();
   }

   public ArrayList<GDXOpenCVVideoVisualizer> getImagePanels()
   {
      return imagePanels;
   }
}
