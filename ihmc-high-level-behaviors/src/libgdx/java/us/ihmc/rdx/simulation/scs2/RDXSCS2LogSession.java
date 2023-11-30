package us.ihmc.rdx.simulation.scs2;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.LogVideoLoader;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.UnitConversions;

import java.io.IOException;
import java.util.ArrayList;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private final ArrayList<RDXOpenCVVideoVisualizer> imagePanels = new ArrayList<>();

   private final LogSession session;
   private LogVideoLoader logVideoLoader;

   // TODO: Use this for loading perception logs sync'ed with robot timestamps
   private PerceptionDataLoader loader;


   public RDXSCS2LogSession(RDXBaseUI baseUI, LogSession session)
   {
      super(baseUI);

      super.startSession(session);
      this.session = session;
      dtHz.set((int) UnitConversions.secondsToHertz(session.getSessionDTSeconds()));

      imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
      imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaSouth", false));

      for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
      {
         onSessionStartedRunnable.run();
      }

      for (RDXOpenCVVideoVisualizer visualizer : imagePanels)
      {
         baseUI.getPrimaryScene().addRenderableProvider(visualizer);
      }
   }

   public void createLogVideoLoader(String logVideoFilePath, String logVideoTimestampFilePath) throws IOException
   {
      logVideoLoader = new LogVideoLoader(logVideoFilePath, logVideoTimestampFilePath);
   }

   public void createPerceptionDataLoader(String perceptionLogFile)
   {
      loader = new PerceptionDataLoader();
      loader.openLogFile(perceptionLogFile);
   }

   public void update()
   {
      super.update();

      if (session.getActiveMode() == SessionMode.RUNNING)
      {
         Mat mat = logVideoLoader.loadNextFrameAsOpenCVMat(session.getLogDataReader().getTimestamp().getValue());

         if (mat != null)
         {
            imagePanels.get(0).setImage(mat);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      renderImGuiWidgetsPartOne();
      renderImGuiWidgetsPartTwo();

      for (RDXOpenCVVideoVisualizer visualizer : imagePanels)
      {
         visualizer.renderImGuiWidgets();
      }
   }

   public void destroy()
   {
      loader.closeLogFile();
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
