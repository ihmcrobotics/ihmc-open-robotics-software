package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.logging.PerceptionDataLoader;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXMocapVisualizer;
import us.ihmc.gdx.ui.graphics.live.GDXVideoVisualizer;
import us.ihmc.gdx.ui.graphics.live.LogVideoLoader;
import us.ihmc.log.LogTools;
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
   private final ArrayList<GDXVideoVisualizer> imagePanels = new ArrayList<>();

   private int cloudIndex = 0;

   public GDXSCS2LogSession(LogSession session)
   {
      super(session);
      this.session = session;
      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));

      imagePanels.add(new GDXVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
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

      for (GDXVideoVisualizer visualizer : imagePanels)
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
            LogTools.info("Point Cloud Render: {}", cloudIndex / 100);
            loader.loadPointCloud("/os_cloud_node/points", cloudIndex / 100, points);
            pointCloudRenderer.setPointsToRender(points);
            if (!points.isEmpty())
            {
               pointCloudRenderer.updateMesh();
            }
         }

         Mat mat = logVideoLoader.loadNextFrameAsOpenCVMat(session.getLogDataReader().getTimestamp().getValue());

         if (mat != null)
         {
            imagePanels.get(0).setImage(mat);
            imagePanels.get(0).update();
         }
         else
         {
            LogTools.warn("Mat is NULL!");
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

      for (GDXVideoVisualizer visualizer : imagePanels)
      {
         visualizer.renderImGuiWidgets();
      }
   }

   protected void renderImGuiWidgetsPartOne()
   {
      super.renderImGuiWidgetsPartOne();
   }

   public ArrayList<GDXVideoVisualizer> getImagePanels()
   {
      return imagePanels;
   }
}
