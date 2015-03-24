package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.movies.MovieFileFilter;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class YoVariableLogVisualizerGUI extends JPanel
{
   private static final long serialVersionUID = 6414551517161321596L;

   private final MultiVideoDataPlayer multiPlayer;
   private final YoVariableLogPlaybackRobot robot;
   private final SimulationConstructionSet scs;
   private YoVariableLogCropper yoVariableLogCropper;

   private final File directory;

   private final Object seekLock = new Object();
   private boolean isSeeking = false;

   public YoVariableLogVisualizerGUI(File directory, LogProperties properties, MultiVideoDataPlayer player, YoVariableLogPlaybackRobot robot,
         YoVariableLogCropper yoVariableLogCropper, SimulationConstructionSet scs)
   {
      super();

      this.multiPlayer = player;
      this.robot = robot;
      this.yoVariableLogCropper = yoVariableLogCropper;
      this.scs = scs;
      this.directory = directory;

      setLayout(new GridLayout(1, 2));

      addGUIElements(directory, properties);

      setVisible(true);
   }

   private void switchVideoUpdate(String videoName)
   {
      if (multiPlayer != null)
         multiPlayer.setActivePlayer(videoName);

   }

   private void seek(int newValue)
   {
      synchronized (seekLock)
      {
         if (!isSeeking && !scs.isSimulating())
         {
            robot.seek(newValue-1); //Do -1 so that we'll get to sliderValue after doing the seek.

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
   }

   private void exportVideo(int start, int end)
   {
      if (start == -1 || end == -1)
      {
         return;
      }

      if (multiPlayer != null)
      {

         final long startTimestamp = robot.getTimestamp(start);
         final long endTimestamp = robot.getTimestamp(end);

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

   private void crop(int start, int end)
   {
      if (start == -1 || end == -1)
      {
         return;
      }
      if (yoVariableLogCropper != null)
      {

         final long startTimestamp = robot.getTimestamp(start);
         final long endTimestamp = robot.getTimestamp(end);

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
      else
      {
         JOptionPane.showMessageDialog(null,
               "Cannot crop this data file. Only compressed log files are supported. Use LogCompressor to compress this log file.", "Cannot crop",
               JOptionPane.ERROR_MESSAGE);
      }
   }

   private void addGUIElements(File directory, LogProperties properties)
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
      if (multiPlayer != null && multiPlayer.getVideos().size() > 0)
      {
         videoFiles.setSelectedIndex(1);
      }

      //    timePanel.add(new JSeparator(JSeparator.VERTICAL));

      JPanel timePanel = new JPanel();
      timePanel.setLayout(new BorderLayout());
      timePanel.add(new JLabel("Position: "), BorderLayout.WEST);

      //    timePanel.add(new JLabel(String.valueOf(robot.getInitialTimestamp())));
      final MarkableJSlider slider = new MarkableJSlider(0, robot.getNumberOfEntries(), 0);

      final JLabel currentTime = new JLabel(String.valueOf(robot.getFinalTimestamp()));

      slider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            int sliderValue = slider.getValue();
            
            final String val = String.valueOf(sliderValue);
            SwingUtilities.invokeLater(new Runnable()
            {

               @Override
               public void run()
               {
                  currentTime.setText(val);
               }
            });
            seek(sliderValue);

         }
      });

      robot.addCurrentRecordTickListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            synchronized (seekLock)
            {
               isSeeking = true;
               slider.setValue((int) ((IntegerYoVariable) v).getIntegerValue());
               isSeeking = false;
            }
         }
      });

      final JButton startMark = new JButton("Mark start");
      startMark.setToolTipText("Set mark to start");
      startMark.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.markStart();
         }
      });

      final JButton endMark = new JButton("Mark end");
      endMark.setToolTipText("Set mark to end");
      endMark.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.markEnd();
         }
      });
      final JButton clearMark = new JButton("Clear mark");
      clearMark.setToolTipText("Clear mark on slider");
      clearMark.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.clear();
         }
      });

      final JButton exportVideo = new JButton("Export video");
      exportVideo.setToolTipText("Export video starting from the in point till the out point.");
      exportVideo.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportVideo(slider.getStart(), slider.getEnd());
         }
      });

      final JButton crop = new JButton("Crop data");
      crop.setToolTipText("Crop starting from the in point till the out point.");
      crop.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            crop(slider.getStart(), slider.getEnd());
         }
      });

      timePanel.add(slider, BorderLayout.CENTER);
      timePanel.add(currentTime, BorderLayout.EAST);

      JPanel dataPanel = new JPanel(new GridLayout(1, 3));
      dataPanel.add(new JLabel("Main class name: " + properties.getLogName()));
      dataPanel.add(new JLabel("Record time: " + properties.getTimestamp()));
      dataPanel.add(new JLabel("Directory: " + directory));
      add(dataPanel);

      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
      add(timePanel);

      JPanel subPanel = new JPanel();
      subPanel.setLayout(new BoxLayout(subPanel, BoxLayout.X_AXIS));
      subPanel.add(new JLabel("Show camera: "));
      subPanel.add(videoFiles);

      subPanel.add(startMark);
      subPanel.add(endMark);
      subPanel.add(clearMark);
      subPanel.add(exportVideo);
      subPanel.add(crop);
      add(subPanel);
   }
}
