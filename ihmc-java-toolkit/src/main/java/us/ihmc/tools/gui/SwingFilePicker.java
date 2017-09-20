package us.ihmc.tools.gui;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.filechooser.FileFilter;

public class SwingFilePicker extends JPanel
{
   private static final long serialVersionUID = -6158977155755198479L;
   private JTextField textField;
   private JButton button;

   private JFileChooser fileChooser;

   public enum Mode
   {
      MODE_OPEN, MODE_SAVE
   }

   private final Mode mode;

   public SwingFilePicker(Mode mode, int width)
   {
      this.mode = mode;
      fileChooser = new JFileChooser();

      setLayout(new FlowLayout(FlowLayout.LEFT, 0, 0));

      textField = new JTextField(width);
      button = new JButton("Browse...");

      button.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            browse(e);
         }
      });

      add(textField);
      add(button);

   }

   private void browse(ActionEvent e)
   {
      File file = new File(textField.getText());
      File parent = file.getAbsoluteFile().getParentFile();
      if (parent.exists())
      {
         fileChooser.setCurrentDirectory(parent);
      }

      if (mode == Mode.MODE_OPEN)
      {
         if (fileChooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION)
         {
            textField.setText(fileChooser.getSelectedFile().getAbsolutePath());
         }
      }
      else if (mode == Mode.MODE_SAVE)
      {
         if (fileChooser.showSaveDialog(this) == JFileChooser.APPROVE_OPTION)
         {
            textField.setText(fileChooser.getSelectedFile().getAbsolutePath());
         }
      }
   }

   public void setAcceptAllFileFilterUsed(boolean b)
   {
      fileChooser.setAcceptAllFileFilterUsed(b);
   }

   public void addFileTypeFilter(FileFilter fileFilter)
   {
      fileChooser.addChoosableFileFilter(fileFilter);
   }

   public void addFileTypeFilter(String extension, String description)
   {
      MyFileFilter filter = new MyFileFilter(extension, description);
      addFileTypeFilter(filter);
   }

   public void setFileSelectionMode(int fileSelectionMode)
   {
      fileChooser.setFileSelectionMode(fileSelectionMode);
   }
   
   public String getSelectedFilePath()
   {
      return textField.getText();
   }

   public void setSelectedFilePath(String path)
   {
      textField.setText(path);
   }

   public JFileChooser getFileChooser()
   {
      return this.fileChooser;
   }
}
