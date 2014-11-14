package us.ihmc.simulationconstructionset.movies;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import org.openh264.EUsageType;

import us.ihmc.codecs.builder.MP4H264MovieBuilder;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.utilities.MemoryTools;

import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ExportMovieCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.XMLReaderUtility;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;

public class ExportMovie implements ExportMovieCommandExecutor
{
   private final static boolean DEBUG = false;

   private final TimeHolder timeHolder;
   private final StandardSimulationGUI standardSimulationGUI;

   private final SimulationSynchronizer simulationSynchronizer;

   private final DataBufferCommandsExecutor dataBufferCommandsExecutor;
   private final GUIEnablerAndDisabler guiEnablerAndDisabler;
   private final RunCommandsExecutor runCommandsExecutor;

   private ActiveCanvas3DHolder captureDeviceHolder;

   private double frameRate = 1.0;

   private double playBackRate = 1.0;

   public ExportMovie(TimeHolder timeHolder, StandardSimulationGUI standardSimulationGUI, DataBufferCommandsExecutor dataBufferCommandsExecutor,
         RunCommandsExecutor runCommandsExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler, ActiveCanvas3DHolder activeCanvas3DHolder,
         SimulationSynchronizer simulationSynchronizer)
   {
      this.timeHolder = timeHolder;
      this.simulationSynchronizer = simulationSynchronizer;
      this.standardSimulationGUI = standardSimulationGUI;

      this.dataBufferCommandsExecutor = dataBufferCommandsExecutor;
      this.runCommandsExecutor = runCommandsExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      this.captureDeviceHolder = activeCanvas3DHolder;
   }

   public void createMovie(File selected)
   {
      this.createMovie(selected, false);
   }

   public void createMovie(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate)
   {
      printIfDebug("Creating Movie. File = " + selected);

      this.frameRate = frameRate;
      this.playBackRate = playBackRate;

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
         moviePlaybackAsBufferedImage(selectedFile.getAbsolutePath(), captureDevice);

         //         buffer = canvas3D.getBufferedImage();
      }

      runCommandsExecutor.setPlaybackRealTimeRate(realTimePlaybackRate);
      guiEnablerAndDisabler.enableGUIComponents();

   }

   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }

   public void createMovie(File selected, Boolean isSequanceSelected)
   {
      createMovie(captureDeviceHolder.getActiveCaptureDevice(), selected, isSequanceSelected, 1.0, 30);
   }

   public void moviePlaybackAsBufferedImage(String file, CaptureDevice captureDevice)
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

      BufferedImage tmp = captureDevice.exportSnapshotAsBufferedImage();

      int bitrate = tmp.getWidth() * tmp.getHeight() * 5; // Heuristic bitrate
      
      MP4H264MovieBuilder movieBuilder = null;
      try
      {
         movieBuilder = new MP4H264MovieBuilder(new File(file), tmp.getWidth(), tmp.getHeight(), (int) frameRate, bitrate,
               EUsageType.CAMERA_VIDEO_REAL_TIME);
   
         movieBuilder.encodeFrame(tmp);
   
         dataBufferCommandsExecutor.gotoInPoint();
         boolean reachedEndPoint = false; // This keeps track of what the previous index was to stop the playback when it starts to loop back.
   
         while (!reachedEndPoint)
         {
            printIfDebug("ExportMovie: Capturing Frame");
            if (DEBUG)
               MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " capturing frame: ");
   
      
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
         e.printStackTrace();
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
