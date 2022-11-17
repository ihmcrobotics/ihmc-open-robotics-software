package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.live.LogVideoLoader;
import us.ihmc.rdx.ui.graphics.live.RDXOpenCVVideoVisualizer;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.UnitConversions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private LogSession session;

   private LogVideoLoader logVideoLoader;
   private PerceptionDataLoader loader;

   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(200000, Point3D32::new);
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private final ArrayList<RDXOpenCVVideoVisualizer> imagePanels = new ArrayList<>();

   private int cloudIndex = 0;

   public RDXSCS2LogSession(LogSession session)
   {
      super.startSession(session);
      this.session = session;
      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));

      imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
      imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaSouth", false));
   }

   public void createLogVideoLoader(String logVideoFilePath, String logVideoTimestampFilePath) throws IOException
   {
      logVideoLoader = new LogVideoLoader(logVideoFilePath, logVideoTimestampFilePath);
   }

   public void createPerceptionDataLoader(String perceptionLogFile)
   {
      loader = new PerceptionDataLoader(perceptionLogFile);
   }

   public void create(RDXBaseUI baseUI)
   {
      super.create(baseUI);

      baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
      pointCloudRenderer.create(2048 * 64, 1);

      for (RDXOpenCVVideoVisualizer visualizer : imagePanels)
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

         if (mat != null)
         {
            imagePanels.get(0).setImage(mat);
         }
      }

      cloudIndex++;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);
   }

   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPartOne();
      renderImGuiWidgetsPartTwo();

      for (RDXOpenCVVideoVisualizer visualizer : imagePanels)
      {
         visualizer.renderImGuiWidgets();
      }
   }

   protected void renderImGuiWidgetsPartOne()
   {
      super.renderImGuiWidgetsPartOne();
   }

   public ArrayList<RDXOpenCVVideoVisualizer> getImagePanels()
   {
      return imagePanels;
   }
}
