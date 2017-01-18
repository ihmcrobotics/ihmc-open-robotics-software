package us.ihmc.simulationconstructionset.videos;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import us.ihmc.codecs.builder.MP4H264MovieBuilder;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ExportVideoCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class ExportVideo implements ExportVideoCommandExecutor
{
   private final static boolean DEBUG = false;

   private final TimeHolder timeHolder;
   private final StandardSimulationGUI standardSimulationGUI;

   private final SimulationSynchronizer simulationSynchronizer;

   private final DataBufferCommandsExecutor dataBufferCommandsExecutor;
   private final GUIEnablerAndDisabler guiEnablerAndDisabler;
   private final RunCommandsExecutor runCommandsExecutor;


   public ExportVideo(TimeHolder timeHolder, StandardSimulationGUI standardSimulationGUI, DataBufferCommandsExecutor dataBufferCommandsExecutor,
         RunCommandsExecutor runCommandsExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler, ActiveCanvas3DHolder activeCanvas3DHolder,
         SimulationSynchronizer simulationSynchronizer)
   {
      this.timeHolder = timeHolder;
      this.simulationSynchronizer = simulationSynchronizer;
      this.standardSimulationGUI = standardSimulationGUI;

      this.dataBufferCommandsExecutor = dataBufferCommandsExecutor;
      this.runCommandsExecutor = runCommandsExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
   }

   public void createVideo(File selectedFile)
   {
      Dimension dimension = new Dimension(1280, 720); // Default to 720p
//      Dimension dimension = new Dimension(1920, 1080); // Default to 1080p

      Boolean isSequanceSelected = false;
      double playBackRate = 1.0;
      double frameRate = 30.0;

      CameraController cameraController = standardSimulationGUI.getActiveView().getCameraController();

      this.createVideo(cameraController, selectedFile, dimension, isSequanceSelected, playBackRate, frameRate);
   }

   public void createVideo(CameraController cameraController, File selectedFile, Dimension dimension, Boolean isSequanceSelected, double playBackRate, double frameRate)
   {
      Graphics3DAdapter graphics3dAdapter = standardSimulationGUI.getGraphics3dAdapter();

      ViewportAdapter adapter = graphics3dAdapter.createNewViewport(null, false, true);

      adapter.setupOffscreenView((int) dimension.getWidth(), (int) dimension.getHeight());

      adapter.setCameraController(cameraController);

      CaptureDevice captureDevice = adapter.getCaptureDevice();
      this.createVideo(captureDevice, selectedFile, false, playBackRate, frameRate);

      graphics3dAdapter.closeViewport(adapter);
   }

   public void createVideo(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate)
   {
      printIfDebug("Creating Video. File = " + selected);

      Vector<BufferedImage> imageVector = new Vector<BufferedImage>();
      int currentTick = 1;
      File selectedFile = selected;

      double realTimePlaybackRate = runCommandsExecutor.getPlaybackRealTimeRate();

      guiEnablerAndDisabler.disableGUIComponents();
      runCommandsExecutor.setPlaybackRealTimeRate(1.0);
      String fileName = selectedFile.getName();

      if (!fileName.contains("."))
      {
         fileName = fileName.concat(".mov");

         if (!selectedFile.getName().equals(fileName))
         {
            File newChosenFile = new File(selectedFile.getParent(), fileName);

            selectedFile = newChosenFile;
         }
      }

      // stop the simulation
      runCommandsExecutor.stop();
      dataBufferCommandsExecutor.gotoOutPoint();

      // go to the start
      dataBufferCommandsExecutor.gotoInPoint();

      // sleep for a little
      try
      {
         Thread.sleep(500);
      }
      catch (InterruptedException e1)
      {
         e1.printStackTrace();
      }

      // record the start tick
      currentTick = dataBufferCommandsExecutor.getInPoint();

      // TICKS_PER_PLAY_CYCLE = sim.getTicksPerPlayCycle();

      String fileNameNoExtension = fileName.substring(0, XMLReaderUtility.getEndIndexOfSubString(0, fileName, ".") - 1);
      dataBufferCommandsExecutor.setIndex(currentTick);

      try
      {
         Thread.sleep(10);
      }
      catch (InterruptedException e)
      {
      }

      Graphics3DAdapter graphics3dAdapter = standardSimulationGUI.getGraphics3dAdapter();
      graphics3dAdapter.play();


      //      BufferedImage buffer = canvas3D.getBufferedImage();

      if (isSequenceSelected)
      {
         //         buffer = canvas3D.getBufferedImage();
         saveSimulationAsSequenceOfImages(selectedFile.getParent(), fileNameNoExtension, captureDevice);
         guiEnablerAndDisabler.enableGUIComponents();
         System.out.println("Finished making saving sequence of Images.");
         imageVector.clear();

         return;
      }
      else
      {
         videoPlaybackAsBufferedImage(selectedFile.getAbsolutePath(), captureDevice, playBackRate, frameRate);

         //         buffer = canvas3D.getBufferedImage();
      }

      graphics3dAdapter.pause();

      runCommandsExecutor.setPlaybackRealTimeRate(realTimePlaybackRate);
      guiEnablerAndDisabler.enableGUIComponents();

   }

   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }

   public void videoPlaybackAsBufferedImage(String file, CaptureDevice captureDevice, double playBackRate, double frameRate)
   {

      // *****************************
      // long nextWakeMillis;
      // long currentTime;

      standardSimulationGUI.updateGraphs();

      try
      {
         Thread.sleep(125);
      }
      catch (InterruptedException e)
      {
      }

      dataBufferCommandsExecutor.gotoInPoint();
      BufferedImage bufferedImage = captureDevice.exportSnapshotAsBufferedImage();
      int bitrate = bufferedImage.getWidth() * bufferedImage.getHeight() / 100; // Heuristic bitrate in kbit/s

      MP4H264MovieBuilder movieBuilder = null;
      try
      {
         movieBuilder = new MP4H264MovieBuilder(new File(file), bufferedImage.getWidth(), bufferedImage.getHeight(), (int) frameRate, bitrate,
               EUsageType.CAMERA_VIDEO_REAL_TIME);

         movieBuilder.encodeFrame(bufferedImage);

         boolean reachedEndPoint = false; // This keeps track of what the previous index was to stop the playback when it starts to loop back.

         while (!reachedEndPoint)
         {
            printIfDebug("ExportVideo: Capturing Frame");


            movieBuilder.encodeFrame(captureDevice.exportSnapshotAsBufferedImage());

            printIfDebug("Waiting For simulationSynchronizer 1");
            synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
            {
               printIfDebug("Done Waiting For simulationSynchronizer 1");

               double currentTime = timeHolder.getTime();
               while(timeHolder.getTime() < currentTime + playBackRate/frameRate && !reachedEndPoint)
               {
                  reachedEndPoint = dataBufferCommandsExecutor.tick(1);
               }
               standardSimulationGUI.updateRobots();
               standardSimulationGUI.updateGraphs();
               standardSimulationGUI.allowTickUpdatesNow();
            }

            //         if (sim.isGraphsUpdatedDuringPlayback())
            standardSimulationGUI.updateGraphs();
         }
      }
      catch(IOException e)
      {
         PrintTools.error(this, "Could not crate movie.  " + e.getMessage());
      }
      finally
      {
         if(movieBuilder != null)
         {
            try
            {
               movieBuilder.close();
            }
            catch (IOException e)
            {
            }
         }
      }
   }

   public Vector<File> saveSimulationAsSequenceOfImages(String path, String NameNoExtension, CaptureDevice captureDevice)
   {
      standardSimulationGUI.updateGraphs();

      try
      {
         Thread.sleep(125);
      }
      catch (InterruptedException e)
      {
      }

      Vector<File> output = new Vector<File>();

      int last = 0; // This keeps track of what the previous index was to stop the playback when it starts to loop back.
      if (standardSimulationGUI == null)
         return null; // Only works with a GUI

      dataBufferCommandsExecutor.gotoInPoint();

      while (last < dataBufferCommandsExecutor.getOutPoint())
      {
         last = dataBufferCommandsExecutor.getIndex();

         File file = new File(path, NameNoExtension + "_" + last + ".jpeg");

         captureDevice.exportSnapshot(file);
         output.add(file);

         printIfDebug("Waiting For simulationSynchronizer 2");

         synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
         {
            printIfDebug("Done Waiting For simulationSynchronizer 2");

            dataBufferCommandsExecutor.tick(1);
            standardSimulationGUI.updateRobots();
            standardSimulationGUI.allowTickUpdatesNow();
         }

         //         if (updateGraphs)
         standardSimulationGUI.updateGraphs();
      }

      return output;
   }

   // /**
   // * This is almost exactly like playCycle() but it also saves the screenShots of each frame as it plays the simulation.
   // */
   // public Vector<BufferedImage> moviePlaybackAsBufferedImage(YoCanvas3D canvas3D)
   // {
   // myGUI.updateGraphs();
   //
   // try
   // {
   // Thread.sleep(125);
   // }
   // catch (InterruptedException e)
   // {
   // }
   //
   // nextWakeMillis = System.currentTimeMillis();
   // Vector<BufferedImage> output = new Vector<BufferedImage>();
   // last = 0;    // This keeps track of what the previous index was to stop the playback when it starts to loop back.
   // if (myGUI == null)
   // return null;    // Only works with a GUI
   // myDataBuffer.goToInPoint();
   // while (last <= myDataBuffer.getOutPoint())
   // {
   // System.out.println("Here C");
   // last = myDataBuffer.getIndex();
   //
   // //        while ((currentTime = System.currentTimeMillis()) < nextWakeMillis)
   // //        {
   // //           try
   // //           {
   // //              Thread.sleep(5);
   // //           }
   // //           catch (InterruptedException e)
   // //           {
   // //           }
   // //
   // //           // Thread.yield();
   // //        }
   //
   //
   // //      exportSnapshot(file);
   // //      exportSn
   // output.add(exportSnapshotAsBufferedImage(canvas3D));
   //
   // //
   // //        long numTicks = (currentTime - nextWakeMillis) / ((PLAY_CYCLE_TIME_MS)) + 1;
   // //
   // //        nextWakeMillis = nextWakeMillis + ((PLAY_CYCLE_TIME_MS)) * numTicks;
   //
   // // myDataBuffer.tick(Math.max((int) (TICKS_PER_PLAY_CYCLE * numTicks), 1));
   //
   // // myGUI.allowTickUpdatesNow();
   //
   // synchronized (robots)    // Synched so we don't update during a graphics redraw...
   // {
   // myDataBuffer.tick(1);
   // myGUI.updateRobots();
   // myGUI.allowTickUpdatesNow();
   // }
   //
   // if (updateGraphs)
   // myGUI.updateGraphs();
   // }
   //
   // return output;
   // }

}
