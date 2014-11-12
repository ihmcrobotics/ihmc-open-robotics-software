package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.movies.MovieFileFilter;

public class YoVariableLogVisualizerGUI extends JPanel
{
   private static final long serialVersionUID = 6414551517161321596L;

   private final MultiVideoDataPlayer multiPlayer;
   private final YoVariableLogPlaybackRobot robot;
   private final SimulationConstructionSet scs;
   private YoVariableLogCropper yoVariableLogCropper;

   private final File directory;

   public YoVariableLogVisualizerGUI(File directory, MultiVideoDataPlayer player, YoVariableLogPlaybackRobot robot, YoVariableLogCropper yoVariableLogCropper,
         SimulationConstructionSet scs)
   {
      super();

      this.multiPlayer = player;
      this.robot = robot;
      this.yoVariableLogCropper = yoVariableLogCropper;
      this.scs = scs;
      this.directory = directory;

      setLayout(new GridLayout(1, 2));

      addGUIElements();

      setVisible(true);
   }

   private void switchVideoUpdate(String videoName)
   {
      if (multiPlayer != null)
         multiPlayer.setActivePlayer(videoName);

   }

   private void seek(int newValue)
   {
      if (!scs.isSimulating())
      {
         robot.seek(newValue);

         try
         {
            scs.simulateOneRecordStepNow();
            scs.setInPoint();
         }
         catch (UnreasonableAccelerationException e)
         {
            e.printStackTrace();
         }

         if (multiPlayer != null)
            multiPlayer.indexChanged(0, 0);
      }
   }

   private long[] getInAndOut()
   {
      if (scs.isSimulating())
      {
         scs.stop();
      }

      scs.gotoInPointNow();
      long startTimestamp = -1;
      if (multiPlayer != null)
         startTimestamp = multiPlayer.getCurrentTimestamp();
      scs.gotoOutPointNow();
      long endTimestamp = -1;
      if (multiPlayer != null)
         endTimestamp = multiPlayer.getCurrentTimestamp();

      if (startTimestamp > -1 && endTimestamp > -1 && startTimestamp > endTimestamp)
      {

         JOptionPane.showMessageDialog(this, "startTimestamp > endTimestamp. Please set the in-point and out-point correctly", "Timestmap error",
               JOptionPane.ERROR_MESSAGE);

         return new long[0];
      }

      return new long[] { startTimestamp, endTimestamp };
   }

   private void exportVideo()
   {
      if (multiPlayer != null)
      {
         long[] timestamps = getInAndOut();
         if (timestamps.length != 2)
         {
            return;
         }

         final long startTimestamp = timestamps[0];
         final long endTimestamp = timestamps[1];

         FileDialog saveDialog = new FileDialog((Frame) null, "Export video", FileDialog.SAVE);

         MovieFileFilter filter = new MovieFileFilter();
         saveDialog.setFilenameFilter(filter);
         saveDialog.setVisible(true);

         String file = saveDialog.getFile();
         if (file != null)
         {
            final File selectedFile = new File(saveDialog.getDirectory(), saveDialog.getFile());
            if (!filter.accept(selectedFile))
            {
               JOptionPane.showMessageDialog(scs.getJFrame(), "Unknown file type \"" + saveDialog.getFile() + "\"\nExcpected: \n" + filter.getDescription());
            }
            else
            {
               new Thread()
               {
                  @Override
                  public void run()
                  {
                     multiPlayer.exportCurrentVideo(selectedFile, startTimestamp, endTimestamp);
                  }
               }.start();
            }
         }
      }
   }

   private void crop()
   {
      long[] timestamps = getInAndOut();
      if (timestamps.length != 2)
      {
         return;
      }

      final long startTimestamp = timestamps[0];
      final long endTimestamp = timestamps[1];

      JFileChooser saveDialog = new JFileChooser(directory);
      saveDialog.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);

      if (JFileChooser.APPROVE_OPTION == saveDialog.showSaveDialog(this))
      {

         final File selectedFile = saveDialog.getSelectedFile();
         new Thread()
         {
            @Override
            public void run()
            {
               yoVariableLogCropper.crop(selectedFile, startTimestamp, endTimestamp);
            }
         }.start();
      }
   }

   private void addGUIElements()
   {
      final JComboBox<String> videoFiles = new JComboBox<>();
      videoFiles.addItem("-- Disabled --");

      if (multiPlayer != null)
      {
         for (String video : multiPlayer.getVideos())
         {
            videoFiles.addItem(video);
         }
      }
      videoFiles.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (videoFiles.getSelectedIndex() == 0)
            {
               switchVideoUpdate(null);
            }
            else
            {
               switchVideoUpdate((String) videoFiles.getSelectedItem());
            }
         }
      });
      if (multiPlayer != null && multiPlayer.getVideos().length > 0)
      {
         videoFiles.setSelectedIndex(1);
      }

      //    timePanel.add(new JSeparator(JSeparator.VERTICAL));

      JPanel timePanel = new JPanel();
      timePanel.setLayout(new BorderLayout());
      timePanel.add(new JLabel("Position: "), BorderLayout.WEST);

      //    timePanel.add(new JLabel(String.valueOf(robot.getInitialTimestamp())));
      final JSlider slider = new JSlider(0, robot.getNumberOfEntries(), 0);

      final JLabel currentTime = new JLabel(String.valueOf(robot.getFinalTimestamp()));

      slider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            currentTime.setText(slider.getValue() + "");
            seek(slider.getValue());
         }
      });

      robot.addCurrentRecordTickListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (scs.isSimulating())
            {
               slider.setValue((int) ((LongYoVariable) v).getLongValue());
            }
         }
      });

      final JButton exportVideo = new JButton("Export video");
      exportVideo.setToolTipText("Export video starting from the in point till the out point.");
      exportVideo.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportVideo();
         }
      });

      final JButton crop = new JButton("Crop data");
      crop.setToolTipText("Crop starting from the in point till the out point.");
      crop.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            crop();
         }
      });

      timePanel.add(slider, BorderLayout.CENTER);
      timePanel.add(currentTime, BorderLayout.EAST);
      setLayout(new BorderLayout());
      add(timePanel, BorderLayout.NORTH);
      
      JPanel subPanel = new JPanel(new GridLayout(1, 3));
      subPanel.add(videoFiles);
      subPanel.add(exportVideo);
      subPanel.add(crop);
      add(subPanel);
   }
}
