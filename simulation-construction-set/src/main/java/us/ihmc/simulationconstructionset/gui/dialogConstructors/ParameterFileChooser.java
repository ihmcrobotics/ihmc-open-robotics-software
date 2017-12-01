package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.Component;
import java.awt.GridLayout;
import java.io.File;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;

import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import us.ihmc.tools.gui.MyFileFilter;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterFileChooser
{
   private final JTextField rootPath;

   private JFileChooser fileChooser;

   private List<YoVariableRegistry> registries;
   private File parameterFile;

   private NameSpace defaultRoot = null;
   private File parameterFilePath = null;

   public ParameterFileChooser()
   {
      rootPath = new JTextField("", 30);

      JPanel extraPanel = new JPanel();
      extraPanel.setLayout(new GridLayout(2, 1));

      extraPanel.add(new JLabel("Parameter root path:"));

      JPanel textHolder = new JPanel();
      textHolder.add(rootPath);
      rootPath.setAlignmentY(Component.TOP_ALIGNMENT);
      extraPanel.add(textHolder);

      fileChooser = new JFileChooser(new File(System.getProperty("user.dir")));
      fileChooser.setFileFilter(new MyFileFilter("xml", "XML Files"));
      fileChooser.setAccessory(extraPanel);
   }

   public boolean showDialog(Component parent, YoVariableRegistry registry, NameSpace newDefaultRoot, File file, boolean save)
   {
      if (file != null && file != parameterFilePath) // Default path changed. Browse to this file
      {
         parameterFilePath = file;
         fileChooser.setSelectedFile(parameterFilePath);
      }

      if (newDefaultRoot != defaultRoot)
      {
         if (newDefaultRoot == null)
         {
            rootPath.setText("");
         }
         else
         {
            rootPath.setText(newDefaultRoot.getName());
         }
         defaultRoot = newDefaultRoot;
      }

      int returnVal;

      if (save)
      {
         returnVal = fileChooser.showSaveDialog(parent);
      }
      else
      {
         returnVal = fileChooser.showOpenDialog(parent);
      }

      if (returnVal == JFileChooser.APPROVE_OPTION)
      {

         if (rootPath.getText().trim().isEmpty())
         {
            registries = Collections.singletonList(registry);
         }
         else
         {
            NameSpace fullNameSpace;

            try
            {
               fullNameSpace = new NameSpace(rootPath.getText().trim());
            }
            catch (RuntimeException e)
            {
               JOptionPane.showMessageDialog(parent, "Invalid namespace. " + e.getMessage(), "Invalid namespace", JOptionPane.ERROR_MESSAGE);
               return false;
            }

            try
            {
               YoVariableRegistry root = registry.getRegistry(fullNameSpace);
               registries = Collections.unmodifiableList(root.getChildren());
            }
            catch (RuntimeException e)
            {
               JOptionPane.showMessageDialog(parent, "Cannot find registry with namespace " + rootPath.getText(), "Unknown root", JOptionPane.ERROR_MESSAGE);
               return false;
            }

         }

         parameterFile = fileChooser.getSelectedFile();

         if (!parameterFile.getName().endsWith(".xml"))
         {
            parameterFile = new File(parameterFile.getParentFile(), parameterFile.getName() + ".xml");
         }

         if (save)
         {
            if (parameterFile.exists() && !parameterFile.canWrite())
            {
               JOptionPane.showMessageDialog(parent, "Cannot write to " + parameterFile.getPath(), "Cannot write to file", JOptionPane.ERROR_MESSAGE);
               return false;
            }
            if (parameterFile.exists())
            {
               return JOptionPane.showOptionDialog(parent, "File exists, overwrite?", "Overwrite?", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE,
                                                   null, null, null) == JOptionPane.YES_OPTION;
            }
            return true;
         }
         else
         {
            if (!parameterFile.canRead())
            {
               JOptionPane.showMessageDialog(parent, "Cannot read " + parameterFile.getPath(), "Cannot read file", JOptionPane.ERROR_MESSAGE);
               return false;

            }
         }

         return true;
      }
      else
      {
         return false;
      }
   }

   public List<YoVariableRegistry> getRegistries()
   {
      return registries;
   }

   public File getFile()
   {
      return parameterFile;
   }

   public void closeAndDispose()
   {
      fileChooser = null;
   }
}
