package us.ihmc.communication.configuration;

import java.awt.Container;
import java.awt.FileDialog;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.EnumMap;
import java.util.Properties;

import javax.sql.rowset.spi.SyncResolver;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.EtchedBorder;

public class NetworkParametersCreator
{
   private final JFrame frame = new JFrame();
   
   private boolean waiting = true;
   private final Object lock = new Object();

   private final EnumMap<NetworkParameterKeys, JTextField> entryBoxes = new EnumMap<>(NetworkParameterKeys.class);

//   private final JTextField exportName;
//   private final JComboBox<NetworkParameterKeys> destination;

   public NetworkParametersCreator()
   {
      File defaultFile = new File(NetworkParameters.defaultParameterFile);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setTitle(getClass().getSimpleName());
      Container content = frame.getContentPane();
      content.setLayout(new BoxLayout(content, BoxLayout.Y_AXIS));

      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         JPanel panel = new JPanel();
         panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
         panel.setBorder(BorderFactory.createTitledBorder(key.toString() + (key.isRequired() ? "*" : "")));
         JLabel description = new JLabel(key.getDescription());
         JTextField host = new JTextField(64);
         host.setText(key.getDefaultValue());
         panel.add(description);
         panel.add(host);

         entryBoxes.put(key, host);

         content.add(panel);
      }

      JPanel requiredPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
      JLabel required = new JLabel("* required");
      requiredPanel.add(required);
      content.add(requiredPanel);

      JPanel savePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
      savePanel.setBorder(BorderFactory.createEtchedBorder(EtchedBorder.LOWERED));

      JButton save = new JButton("Save");
      save.addActionListener(new SaveActionListener(defaultFile, true));
      save.setToolTipText("Save parameters to default location (" + defaultFile.getAbsolutePath() + ")");
      savePanel.add(save);

      JButton load = new JButton("Import...");
      load.addActionListener(new LoadActionListener());
      load.setToolTipText("Import parameters from file");
      savePanel.add(load);

      JButton export = new JButton("Export...");
      export.addActionListener(new SaveActionListener(null, false));
      export.setToolTipText("Export parameters to file");
      savePanel.add(export);

      JButton quit = new JButton("Quit without saving");
      quit.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            synchronized(lock)
            {
               waiting = false;
               lock.notifyAll();
            }
         }
      });
      savePanel.add(quit);
      
      content.add(savePanel);

//      JPanel exportPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
//      exportPanel.setBorder(BorderFactory.createEtchedBorder(EtchedBorder.LOWERED));
//
//      JButton export = new JButton("Export to: ");
//      export.addActionListener(new ExportActionListener());
//      destination = new JComboBox<>(NetworkParameterKeys.values());
//      JLabel nameLabel = new JLabel(" Path:");
//      exportName = new JTextField(NetworkParameters.defaultParameterFile, 32);
//
//      exportPanel.add(export);
//      exportPanel.add(destination);
//      exportPanel.add(nameLabel);
//      exportPanel.add(exportName);
//
//      content.add(exportPanel);
      
      if(defaultFile.exists())
      {
         load(defaultFile);
      }
      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
      System.out.println("WAITING");
      synchronized (lock)
      {
         while (waiting)
         {
            try
            {
               lock.wait();
            }
            catch (InterruptedException e)
            {
            }
         }
      }
      
      frame.setVisible(false);
      frame.dispose();
   }

   private boolean isValid()
   {
      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         if (key.isRequired() && entryBoxes.get(key).getText().length() == 0)
         {
            JOptionPane.showMessageDialog(frame, key.toString() + " is required.", "Missing required fields", JOptionPane.ERROR_MESSAGE);
            return false;
         }
      }
      return true;
   }

   private void load(File file)
   {
      try
      {
         FileInputStream in = new FileInputStream(file);
         Properties properties = new Properties();
         properties.load(in);
         for (NetworkParameterKeys key : NetworkParameterKeys.values())
         {
            String property = properties.getProperty(key.toString());
            if(property != null)
            {
               entryBoxes.get(key).setText(property);
            }
            else
            {
               entryBoxes.get(key).setText("");               
            }
         }
         in.close();
      }
      catch (IOException e)
      {
         JOptionPane.showMessageDialog(frame, "Cannot load " + file, "Read error", JOptionPane.ERROR_MESSAGE);

      }
   }

   private void save(File file, boolean exit)
   {
      try
      {
         FileOutputStream out = new FileOutputStream(file);
         Properties properties = new Properties();
         for (NetworkParameterKeys key : NetworkParameterKeys.values())
         {
            if (entryBoxes.get(key).getText().length() != 0)
            {
               properties.setProperty(key.toString(), entryBoxes.get(key).getText());
            }
         }
         properties.store(out, "Generated by " + getClass().getCanonicalName());
         out.close();
         if(exit)
         {
            synchronized(lock)
            {
               waiting = false;
               lock.notifyAll();
            }
         }
      }
      catch (IOException e)
      {
         JOptionPane.showMessageDialog(frame, "Cannot write to file " + file, "Write error", JOptionPane.ERROR_MESSAGE);
      }
   }

//   private void export(String host, String path)
//   {
//      JOptionPane.showMessageDialog(frame, "TODO: Implement exporting to " + host + ":" + path, "Implement me", JOptionPane.ERROR_MESSAGE);
//
//   }
//
//   private class ExportActionListener implements ActionListener
//   {
//
//      @Override
//      public void actionPerformed(ActionEvent e)
//      {
//         if (isValid())
//         {
//            NetworkParameterKeys key = (NetworkParameterKeys) destination.getSelectedItem();
//            if (entryBoxes.get(key).getText().length() == 0)
//            {
//               JOptionPane.showMessageDialog(frame, key + " is not set.", "Invalid host", JOptionPane.ERROR_MESSAGE);
//               return;
//            }
//
//            if (exportName.getText().length() == 0)
//            {
//               JOptionPane.showMessageDialog(frame, "No path given", "Invalid entry", JOptionPane.ERROR_MESSAGE);
//               return;
//            }
//
//            export(entryBoxes.get(key).getText(), exportName.getText());
//         }
//
//      }
//
//   }

   private class LoadActionListener implements ActionListener
   {

      @Override
      public void actionPerformed(ActionEvent e)
      {
         FileDialog dialog = new FileDialog(frame, "Choose ini file to load", FileDialog.LOAD);
         dialog.setFilenameFilter(new INIFileFilter());
         dialog.setVisible(true);
         String filename = dialog.getFile();
         if (filename == null)
         {
            return;
         }
         else
         {
            load(new File(dialog.getDirectory(), dialog.getFile()));
         }
      }

   }

   private class SaveActionListener implements ActionListener
   {
      
      private final File target;
      private final boolean exit;
      public SaveActionListener(File target, boolean exit)
      {
         this.target = target;      
         this.exit = exit;
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         if (isValid())
         {
            if(target != null)
            {
               save(target, exit);
            }
            else
            {
               FileDialog dialog = new FileDialog(frame, "Choose file", FileDialog.SAVE);
               dialog.setFilenameFilter(new INIFileFilter());
               dialog.setFile(NetworkParameters.defaultParameterFile);
               dialog.setVisible(true);
               
               String filename = dialog.getFile();
               if (filename == null)
               {
                  return;
               }
               else
               {
                  save(new File(dialog.getDirectory(), dialog.getFile()), exit);
               }               
            }
         }
      }

   }

   private final class INIFileFilter implements FilenameFilter
   {
      @Override
      public boolean accept(File dir, String name)
      {
         if (name.endsWith(".ini"))
         {
            return true;
         }
         else
         {
            return false;
         }
      }
   }

   public static void main(String[] args)
   {
      new NetworkParametersCreator();
   }

}
