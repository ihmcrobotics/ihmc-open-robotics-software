package us.ihmc.avatar.scs2;

import org.bytedeco.javacv.OpenCVFrameConverter;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.session.log.*;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Adds video scrubbers to the log session.
 * TODO: Add to upstream SCS 2 LogSession
 */
public class SCS2LogSessionWithVideo extends LogSession
{
   private YoLong yoTimestamp;
   private File logDirectory;
   private LogDataReaderInterface logDataReader;
   private LogPropertiesReader logProperties;

   private final List<MagewellScrubber> magewellScrubbers = new ArrayList<>();
   private final List<BlackMagicScrubber> blackMagicScrubbers = new ArrayList<>();
   private final List<ZEDSVOScrubber> zedSVOScrubbers = new ArrayList<>();

   public SCS2LogSessionWithVideo(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      super(logDirectory, progressConsumer);
   }

   @Override
   public boolean startSessionThread()
   {
      boolean started = super.startSessionThread();

      if (started)
      {
         logDirectory = getLogDirectory();
         logDataReader = getLogDataReader();
         logProperties = getLogProperties();
         yoTimestamp = logDataReader.getTimestamp();

         for (int i = 0; i < logProperties.getCameras().size(); i++)
         {
            Camera camera = logProperties.getCameras().get(i);
            LogTools.info("Found camera: %s".formatted(camera.getName()));
            try
            {
               if (camera.getTypeAsString().equals(CameraType.CAPTURE_CARD_MAGEWELL.toString()))
               {
                  MagewellScrubber magewellScrubber = new MagewellScrubber(camera, logDirectory, logProperties.getVideo().getHasTimebase());
                  magewellScrubbers.add(magewellScrubber);
               }
               else
               {
                  BlackMagicScrubber blackMagicScrubber = new BlackMagicScrubber(camera,
                                                                                 logDirectory,
                                                                                 logProperties.getVideo().getHasTimebase());
                  blackMagicScrubbers.add(blackMagicScrubber);
               }
            }
            catch (IOException e)
            {
               LogTools.error(e.getMessage());
            }
         }

         for (File zedSensorDatFile : ZEDSVOScrubber.findZEDSensorDatFiles(logDirectory))
         {
            LogTools.info("Found ZED sensor: %s".formatted(zedSensorDatFile.getName()));
            ZEDSVOScrubber zedSVOScrubber = new ZEDSVOScrubber(zedSensorDatFile);
            zedSVOScrubbers.add(zedSVOScrubber);
         }

      }

      return started;
   }

   @Override
   public boolean stopSessionThread()
   {
      boolean stopped = super.stopSessionThread();

      if (stopped)
         disposeScrubbers();

      return stopped;
   }

   @Override
   public void shutdownSession()
   {
      super.shutdownSession();
      disposeScrubbers();
   }

   private void disposeScrubbers()
   {
      for (MagewellScrubber magewellScrubber : magewellScrubbers)
         magewellScrubber.getMagewellDemuxer().stop();
      for (BlackMagicScrubber blackMagicScrubber : blackMagicScrubbers)
         blackMagicScrubber.getDemuxer().delete();
      for (ZEDSVOScrubber zedSVOScrubber : zedSVOScrubbers)
         zedSVOScrubber.close();

      magewellScrubbers.clear();
      blackMagicScrubbers.clear();
      zedSVOScrubbers.clear();
   }

   public List<MagewellScrubber> getMagewellScrubbers()
   {
      return magewellScrubbers;
   }

   public List<BlackMagicScrubber> getBlackMagicScrubbers()
   {
      return blackMagicScrubbers;
   }

   public List<ZEDSVOScrubber> getZedSVOScrubbers()
   {
      return zedSVOScrubbers;
   }
}
