package us.ihmc.robotDataVisualizer.logger;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataVisualizer.logger.converters.VideoConverter;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;

import static java.lang.Long.parseLong;

public class VideoDataPlayer
{
   private static final boolean VIDEO_STATISTICS = false;
   private final String name;
   private final boolean hasTimebase;
   private final boolean interlaced;

   private static HideableMediaFrame viewer;

   private long[] robotTimestamps;
   private long[] videoTimestamps;

   private long bmdTimeBaseNum;
   private long bmdTimeBaseDen;

   private final MP4VideoDemuxer demuxer;
   private final YUVPictureConverter converter = new YUVPictureConverter();

   private final File videoFile;

   private int currentlyShowingIndex = 0;
   private long currentlyShowingRobottimestamp = 0;
   private long upcomingRobottimestamp = 0;

   public VideoDataPlayer(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.name = camera.getNameAsString();
      this.interlaced = camera.getInterlaced();
      this.hasTimebase = hasTimeBase;

      if (!hasTimebase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }
      videoFile = new File(dataDirectory, camera.getVideoFileAsString());
      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());

      parseTimestampData(timestampFile);

      demuxer = new MP4VideoDemuxer(videoFile);

      viewer = new HideableMediaFrame(camera.getNameAsString(), demuxer.getWidth(), demuxer.getHeight());
   }

   public synchronized void showVideoFrame(long timestamp)
   {
      if (timestamp >= currentlyShowingRobottimestamp && timestamp < upcomingRobottimestamp)
      {
         return;
      }

      long previousTimestamp = videoTimestamps[currentlyShowingIndex];

      long videoTimestamp;
      long nextVideoTimestamp = videoTimestamps[currentlyShowingIndex];


      if (robotTimestamps.length > currentlyShowingIndex + 1 && robotTimestamps[currentlyShowingIndex + 1] == timestamp)
      {
         currentlyShowingIndex++;
         videoTimestamp = videoTimestamps[currentlyShowingIndex];
         currentlyShowingRobottimestamp = robotTimestamps[currentlyShowingIndex];
      }
      else
      {
         videoTimestamp = getVideoTimestampWithBinarySearch(timestamp);
      }

      if (robotTimestamps.length > currentlyShowingIndex + 1)
      {
         nextVideoTimestamp = videoTimestamps[currentlyShowingIndex + 1];
      }

      if (currentlyShowingIndex + 1 < robotTimestamps.length)
      {
         upcomingRobottimestamp = robotTimestamps[currentlyShowingIndex + 1];
      }
      else
      {
         upcomingRobottimestamp = currentlyShowingRobottimestamp;
      }

      if (previousTimestamp == videoTimestamp)
      {
         return;
      }

      try
      {
         FrameData copyForWriting = new FrameData();
         copyForWriting.cameraTargetPTS = nextVideoTimestamp;
         copyForWriting.cameraCurrentPTS = demuxer.getCurrentPTS();
         copyForWriting.cameraPreviousPTS = previousTimestamp;
         copyForWriting.robotTimestamp = currentlyShowingRobottimestamp;

         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();

         viewer.update(nextFrame, copyForWriting);
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

   }

   public void setVisible(boolean visible)
   {
      viewer.setVisible(visible);
   }

   private long getVideoTimestampWithBinarySearch(long timestamp)
   {
      if (timestamp <= robotTimestamps[0])
      {
         currentlyShowingIndex = 0;
         return videoTimestamps[currentlyShowingIndex];
      }

      if (timestamp >= robotTimestamps[robotTimestamps.length-1])
      {
         currentlyShowingIndex = robotTimestamps.length - 2;
         return videoTimestamps[currentlyShowingIndex];
      }

      currentlyShowingIndex = Arrays.binarySearch(robotTimestamps, timestamp);

      if (currentlyShowingIndex < 0)
      {
         int nextIndex = -currentlyShowingIndex - 1; // insertionPoint
         currentlyShowingIndex = nextIndex;
      }

      return videoTimestamps[currentlyShowingIndex];
   }

   private void parseTimestampData(File timestampFile) throws IOException
   {
      try (BufferedReader reader = new BufferedReader(new FileReader(timestampFile)))
      {
         String line;
         if (hasTimebase)
         {
            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseNum = parseLong(line);
            } else
            {
               throw new IOException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDen = parseLong(line);
            } else
            {
               throw new IOException("Cannot read denumerator");
            }
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();
         while ((line = reader.readLine()) != null)
         {
            String[] stamps = line.split("\\s");
            long robotStamp = parseLong(stamps[0]);
            long videoStamp = parseLong(stamps[1]);

            if (interlaced)
            {
               videoStamp /= 2;
            }

            robotTimestamps.add(robotStamp);
            videoTimestamps.add(videoStamp);
         }

         this.robotTimestamps = robotTimestamps.toArray();
         this.videoTimestamps = videoTimestamps.toArray();

      } catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, ProgressMonitorInterface monitor)
   {

      long startVideoTimestamp = getVideoTimestampWithBinarySearch(startTimestamp);
      long endVideoTimestamp = getVideoTimestampWithBinarySearch(endTimestamp);

      try
      {
         VideoConverter.convert(videoFile, selectedFile, startVideoTimestamp, endVideoTimestamp, monitor);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressMonitorInterface monitor) throws IOException
   {

      long startVideoTimestamp = getVideoTimestampWithBinarySearch(startTimestamp);
      long endVideoTimestamp = getVideoTimestampWithBinarySearch(endTimestamp);

      int frameRate = VideoConverter.crop(videoFile, outputFile, startVideoTimestamp, endVideoTimestamp, monitor);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1);
      timestampWriter.println(frameRate);

      long pts = 0;
      /*
       * PTS gets reorderd to be monotonically increaseing starting from 0
       */
      for (int i = 0; i < robotTimestamps.length; i++)
      {
         long robotTimestamp = robotTimestamps[i];

         if (robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {

            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(pts);
            pts++;
         }
         else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();
   }

   private static class BoldCellRenderer extends DefaultTableCellRenderer
   {
      public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column)
      {
         Component component = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);

         // Check if we are in the first row and BOLD that text
         if (row == 0)
         {
            Font boldFont = new Font(component.getFont().getName(), Font.BOLD, component.getFont().getSize());
            component.setFont(boldFont);

            ((JLabel) component).setHorizontalAlignment(SwingConstants.CENTER);
         }
         else
         {
            // Otherwise don't BOLD anything
            Font plainFont = new Font(component.getFont().getName(), Font.PLAIN, component.getFont().getSize());
            component.setFont(plainFont);

            ((JLabel) component).setHorizontalAlignment(SwingConstants.LEFT);
         }

         return component;
      }
   }


   private class HideableMediaFrame extends JFrame
   {
      private static final long serialVersionUID = -3494797002318746347L;
      private final JLabel label = new JLabel();
      private final JPanel panel = new JPanel();
      private BufferedImage img;
      private int width, height;
      SpringLayout springLayout = new SpringLayout();

      private final Object[][] data = new Object[][]{ {"Timestamp Lable:", "Value:"},
                                                      {" cameraTargetPTS", 0},
                                                      {" cameraCurrentPTS", 0},
                                                      {" cameraPreviousPTS", 0},
                                                      {" robotTimestamp", 0} };

      JTable table = new JTable(data, new String[]{"Timestamp Lable:", "Value: "});

      public HideableMediaFrame(String name, int width, int height)
      {
         super(name);
         this.width = width;
         this.height = height;

         panel.setLayout(springLayout);

         if (VIDEO_STATISTICS)
         {
            panel.add(table);

            table.setBorder(BorderFactory.createLineBorder(Color.GREEN, 2));
            table.setRowHeight(20);
            table.getColumnModel().getColumn(0).setPreferredWidth(42);
            table.setDefaultRenderer(Object.class, new BoldCellRenderer());
            table.setPreferredSize(new Dimension(280, data.length * 20 + 2));

            springLayout.putConstraint(SpringLayout.SOUTH, label,0, SpringLayout.SOUTH, panel);
            springLayout.putConstraint(SpringLayout.WEST, label,0, SpringLayout.WEST, panel);
            springLayout.putConstraint(SpringLayout.SOUTH, table,0, SpringLayout.SOUTH, panel);
            springLayout.putConstraint(SpringLayout.WEST, table,0, SpringLayout.WEST, panel);
         }

         panel.add(label);
         pack();
      }

      public JPanel getPanel()
      {
         return panel;
      }

      public void update(final YUVPicture nextFrame, FrameData timestampData)
      {
         SwingUtilities.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               table.setValueAt(timestampData.cameraTargetPTS, 1, 1);
               table.setValueAt(timestampData.cameraCurrentPTS, 2, 1);
               table.setValueAt(timestampData.cameraPreviousPTS, 3, 1);
               table.setValueAt(timestampData.robotTimestamp, 4, 1);

               img = converter.toBufferedImage(nextFrame, img);
               nextFrame.delete();

               ImageIcon icon = new ImageIcon(img);
               label.setIcon(icon);

               if (img.getWidth() != width || img.getHeight() != height)
               {
                  width = img.getWidth();
                  height = img.getHeight();
                  label.setPreferredSize(new Dimension(width, height));
                  pack();
               }
            }
         });
      }
   }

   public String getName()
   {
      return name;
   }

   public static void main(String[] args) throws IOException
   {
      Camera camera = new Camera();
      camera.setName("test");
      camera.setInterlaced(false);
      String videoName = "ValkyrieTripodNorth";
      camera.setTimestampFile(videoName + "_Timestamps.dat");
      camera.setVideoFile(videoName + "_Video.mov");

      File dataDirectory = new File("//Gideon/LogData/LoggerDevLogs/Comparison/Valkyrie_Capture_SDI_timestamps_3003/");
//      File dataDirectory = new File("//Gideon/LogData/LoggerDevLogs/Comparison/Valkyrie_GStreamer_HDMI_timestamps_3003/");

      VideoDataPlayer player = new VideoDataPlayer(camera, dataDirectory, true);

      for (int i = 1; i < player.robotTimestamps.length; i++)
      {
         if (player.robotTimestamps[i - 1] > player.robotTimestamps[i])
         {
            System.out.println("Non-monotonic robot timestamps");
            System.out.println(player.robotTimestamps[i - 1]);
         }
         if (player.videoTimestamps[i - 1] >= player.videoTimestamps[i])
         {
            System.out.println("Non-monotonic video timestamps");
            System.out.println(player.videoTimestamps[i - 1]);
         }

      }

      viewer.add(viewer.getPanel());
      viewer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
      viewer.setSize(screenSize.width - 100, screenSize.height - 120);

      player.setVisible(true);

      long previousRobotTimestamp = player.robotTimestamps[0];
      long previousNanos = System.nanoTime();
      boolean playRealtime = true;
      
      for (int i = 1; i < player.robotTimestamps.length; i++)
      {
         long nextRobotTimestamp = player.robotTimestamps[i];
         long nextNanos = previousNanos + nextRobotTimestamp - previousRobotTimestamp;

         if (playRealtime)
         {
            while (System.nanoTime() < nextNanos);
         }

         previousNanos = System.nanoTime();

         player.showVideoFrame(nextRobotTimestamp);
         previousRobotTimestamp = nextRobotTimestamp;
      }
   }

   public static class FrameData
   {
      public WritableImage frame;
      public long cameraTargetPTS;
      public long cameraCurrentPTS;
      public long cameraPreviousPTS;
      public long robotTimestamp;
   }
}
