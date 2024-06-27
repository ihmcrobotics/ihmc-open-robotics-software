package us.ihmc.rdx.simulation.scs2;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.LogVideoLoader;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogSession;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private LogSession logSession;

   private LogVideoLoader logVideoLoader;
   private final ArrayList<RDXOpenCVVideoVisualizer> logVideoVisualizers = new ArrayList<>();

   public RDXSCS2LogSession(RDXBaseUI baseUI)
   {
      super(baseUI);
   }

   public RDXSCS2LogSession(RDXBaseUI baseUI, RDXPanel plotManagerParentPanel)
   {
      super(baseUI, plotManagerParentPanel);
   }

   public void startSession(String logFilePath, RDXPerceptionVisualizersPanel perceptionVisualizersPanel)
   {
      logSession = null;

      for (RDXOpenCVVideoVisualizer logVideoVisualizer : logVideoVisualizers)
      {
         perceptionVisualizersPanel.removeVisualizer(logVideoVisualizer);
         logVideoVisualizer.destroy();
      }
      logVideoVisualizers.clear();

      try
      {
         File file = new File(logFilePath);
         LogTools.info("Loading log: {}", file.toPath().getParent().normalize().toAbsolutePath());
         logSession = new LogSession(file.getParentFile(), null);
      }
      catch (IOException e)
      {
         LogTools.error("Failed to load log. {}", e.getMessage());
      }

      if (logSession != null)
      {
         startSession(logSession);


//         imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
//         imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaSouth", false));

         for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
         {
            onSessionStartedRunnable.run();
         }
      }
   }

   public void createLogVideoLoader(String logVideoFilePath, String logVideoTimestampFilePath) throws IOException
   {
      logVideoLoader = new LogVideoLoader(logVideoFilePath, logVideoTimestampFilePath);

//      log
   }

   public void update()
   {
      super.update();

      if (getSession().getActiveMode() == SessionMode.RUNNING)
      {
//         Mat mat = logVideoLoader.loadNextFrameAsOpenCVMat(logSession.getLogDataReader().getTimestamp().getValue());
//
//         if (mat != null)
//         {
//            imagePanels.get(0).setImage(mat);
//         }
      }
   }

   public void destroy()
   {

   }
}
