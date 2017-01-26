package us.ihmc.robotDataLogger.compressor;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.SpringLayout;
import javax.swing.filechooser.FileFilter;

import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataLogger.logger.util.CustomProgressMonitor;
import us.ihmc.tools.gui.SpringUtilities;
import us.ihmc.tools.gui.SwingFilePicker;
import us.ihmc.tools.gui.SwingFilePicker.Mode;

public class LogCompressorUI extends JFrame
{
   private static final long serialVersionUID = -7569157904587962418L;

   private final SwingFilePicker compressionSource = new SwingFilePicker(Mode.MODE_OPEN, 70);
   private final SwingFilePicker compressionTarget = new SwingFilePicker(Mode.MODE_SAVE, 70);

   private final SwingFilePicker decompressionSource = new SwingFilePicker(Mode.MODE_OPEN, 70);
   private final SwingFilePicker decompressionTarget = new SwingFilePicker(Mode.MODE_SAVE, 70);

   private final SwingFilePicker videoConverterCmd = new SwingFilePicker(Mode.MODE_OPEN, 70);
   private final JTextField videoCompressOptions = new JTextField("", 80);
   private final JTextField videoDeCompressOptions = new JTextField("", 80);

   private final JTabbedPane converterPane = new JTabbedPane();
   private final VideoCompressor videoCompressor;

   public LogCompressorUI() throws FileNotFoundException, IOException
   {
      super("Log compressor");
      setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      setLocationByPlatform(true);
      setLocationRelativeTo(null);
      
      this.videoCompressor = new VideoCompressor();

      setOptionsFieldsFromVideoCompressor();

      JPanel compress = createCompressPanel();
      converterPane.addTab("Compress", compress);

      JPanel decompress = createDecompressPanel();
      converterPane.addTab("Decompress", decompress);
      
      JPanel settings = createSettingsPanel();
      converterPane.addTab("Settings", settings);
      

      getContentPane().add(converterPane);
      pack();
      setVisible(true);

   }

   private void setOptionsFieldsFromVideoCompressor()
   {
      videoConverterCmd.setSelectedFilePath(videoCompressor.getVideoCompressorPath());
      videoCompressOptions.setText(videoCompressor.getVideoCompressionOptions());
      videoDeCompressOptions.setText(videoCompressor.getVideoDecompressionOptions());
   }

   private JPanel createCompressPanel()
   {
      JPanel panel = new JPanel(new SpringLayout());

      panel.add(new JLabel("Source:"));
      panel.add(compressionSource);

      compressionSource.setAcceptAllFileFilterUsed(false);
      compressionSource.addFileTypeFilter(new FileFilter()
      {

         @Override
         public String getDescription()
         {
            return YoVariableLoggerListener.propertyFile;
         }

         @Override
         public boolean accept(File f)
         {
            return f.isDirectory() || f.getName().equals(YoVariableLoggerListener.propertyFile);
         }
      });

      compressionTarget.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);

      panel.add(new JLabel("Target:"));
      panel.add(compressionTarget);

      JButton compress = new JButton("Compress");
      compress.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            compress();
         }
      });
      panel.add(new JLabel(""));
      panel.add(compress);

      SpringUtilities.makeCompactGrid(panel, 3, 2, 6, 6, 6, 6);

      return panel;
   }
   
   private JPanel createDecompressPanel()
   {
      JPanel panel = new JPanel(new SpringLayout());
      
      panel.add(new JLabel("Source:"));
      panel.add(decompressionSource);
      
      decompressionSource.setAcceptAllFileFilterUsed(false);
      decompressionSource.addFileTypeFilter(new FileFilter()
      {
         
         @Override
         public String getDescription()
         {
            return "robotData.compressed";
         }
         
         @Override
         public boolean accept(File f)
         {
            return f.isDirectory() || f.getName().equals("robotData.compressed");
         }
      });
      
      decompressionTarget.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      
      panel.add(new JLabel("Target:"));
      panel.add(decompressionTarget);
      
      JButton decompress = new JButton("Decompress");
      decompress.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            decompress();
         }
      });
      panel.add(new JLabel(""));
      panel.add(decompress);
      
      SpringUtilities.makeCompactGrid(panel, 3, 2, 6, 6, 6, 6);
      
      return panel;
   }

   private void compress()
   {
      File sourceFile = new File(compressionSource.getSelectedFilePath());
      final File targetFile = new File(compressionTarget.getSelectedFilePath());

      if (!sourceFile.exists())
      {

         JOptionPane.showMessageDialog(this, "Selected source does not exist", "Invalid source", JOptionPane.ERROR_MESSAGE);
         return;
      }

      if (!sourceFile.getName().equals(YoVariableLoggerListener.propertyFile))
      {
         JOptionPane.showMessageDialog(this, "Selected source is not a log file", "Invalid source", JOptionPane.ERROR_MESSAGE);
         return;
      }

      if (targetFile.equals(""))
      {
         JOptionPane.showMessageDialog(this, "No target directory entered", "Invalid target", JOptionPane.ERROR_MESSAGE);
         return;

      }
      else if (!targetFile.exists())
      {
         if (!targetFile.mkdirs())
         {
            JOptionPane.showMessageDialog(this, "Cannot make target directory " + targetFile, "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;
         }
      }
      else
      {
         if (!targetFile.isDirectory())
         {
            JOptionPane.showMessageDialog(this, "Target is not a directory", "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;

         }
         if (targetFile.list().length > 0)
         {
            JOptionPane.showMessageDialog(this, "Target directory is not empty", "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;
         }
      }

      final CustomProgressMonitor progressMonitor = new CustomProgressMonitor();
      progressMonitor.setEchoToConsole(true);
      final File logDirectory = sourceFile.getParentFile();
      final LogPropertiesReader logProperties = new LogPropertiesReader(new File(logDirectory, YoVariableLoggerListener.propertyFile));
      new Thread()
      {
         @Override
         public void run()
         {
            try
            {
               updateSettings();
               new LogFileCompressor(logDirectory, targetFile, logProperties, progressMonitor);
            }
            catch (IOException e)
            {
               JOptionPane.showMessageDialog(LogCompressorUI.this, e.getMessage(), "Error while compressing log", JOptionPane.ERROR_MESSAGE);
            }

         }
      }.start();
   }
   
   private void decompress()
   {
      File sourceFile = new File(decompressionSource.getSelectedFilePath());
      final File targetFile = new File(decompressionTarget.getSelectedFilePath());
      
      if (!sourceFile.exists())
      {
         
         JOptionPane.showMessageDialog(this, "Selected source does not exist", "Invalid source", JOptionPane.ERROR_MESSAGE);
         return;
      }
      
      if (!sourceFile.getName().equals("robotData.compressed"))
      {
         JOptionPane.showMessageDialog(this, "Selected source is not a compressed log file", "Invalid source", JOptionPane.ERROR_MESSAGE);
         return;
      }
      
      if (targetFile.equals(""))
      {
         JOptionPane.showMessageDialog(this, "No target directory entered", "Invalid target", JOptionPane.ERROR_MESSAGE);
         return;
         
      }
      else if (!targetFile.exists())
      {
         if (!targetFile.mkdirs())
         {
            JOptionPane.showMessageDialog(this, "Cannot make target directory " + targetFile, "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;
         }
      }
      else
      {
         if (!targetFile.isDirectory())
         {
            JOptionPane.showMessageDialog(this, "Target is not a directory", "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;
            
         }
         if (targetFile.list().length > 0)
         {
            JOptionPane.showMessageDialog(this, "Target directory is not empty", "Invalid target", JOptionPane.ERROR_MESSAGE);
            return;
         }
      }
      
      final CustomProgressMonitor progressMonitor = new CustomProgressMonitor();
      progressMonitor.setEchoToConsole(true);
      final File logDirectory = sourceFile.getParentFile();
      final LogPropertiesReader logProperties = new LogPropertiesReader(new File(logDirectory, YoVariableLoggerListener.propertyFile));
      new Thread()
      {
         @Override
         public void run()
         {
            try
            {
               updateSettings();
               new LogFileDecompressor(logDirectory, targetFile, logProperties, progressMonitor);
            }
            catch (IOException e)
            {
               JOptionPane.showMessageDialog(LogCompressorUI.this, e.getMessage(), "Error while compressing log", JOptionPane.ERROR_MESSAGE);
            }
            
         }
      }.start();
   }

   private void loadDefaults()
   {
      videoCompressor.setDefaults();
      setOptionsFieldsFromVideoCompressor();

   }

   private void updateSettings()
   {
      videoCompressor.setVideoCompressorPath(videoConverterCmd.getSelectedFilePath());
      videoCompressor.setVideoCompressionOptions(videoCompressOptions.getText());
      videoCompressor.setVideoDecompressionOptions(videoDeCompressOptions.getText());      
   }
   
   private void saveSettings()
   {
      updateSettings();
      videoCompressor.saveSettings();
   }

   private JPanel createSettingsPanel()
   {
      JPanel settings = new JPanel(new SpringLayout());
      settings.add(new JLabel("Video converter command"));
      settings.add(videoConverterCmd);
      settings.add(new JLabel("Video compression options"));
      settings.add(videoCompressOptions);
      settings.add(new JLabel("Video decompression options"));
      settings.add(videoDeCompressOptions);

      settings.add(new JLabel(""));
      settings.add(new JLabel("{INPUT} gets replaced with the full path to the input video file"));
      settings.add(new JLabel(""));
      settings.add(new JLabel("{OUTPUT} gets replaced with the full path to the output video file"));

      JPanel saveload = new JPanel();
      saveload.setLayout(new BoxLayout(saveload, BoxLayout.LINE_AXIS));
      settings.add(new JLabel(""));
      settings.add(saveload);
      JButton loadDefaults = new JButton("Load defaults");
      loadDefaults.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            loadDefaults();
         }

      });
      saveload.add(loadDefaults);

      JButton save = new JButton("Save");
      save.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            saveSettings();
         }
      });
      saveload.add(save);
      SpringUtilities.makeCompactGrid(settings, 6, 2, 6, 6, 6, 6);
      return settings;
   }

   public static void main(String[] args) throws FileNotFoundException, IOException
   {
      new LogCompressorUI();
   }
}
