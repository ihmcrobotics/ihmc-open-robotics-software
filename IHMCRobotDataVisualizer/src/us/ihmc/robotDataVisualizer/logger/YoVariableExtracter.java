package us.ihmc.robotDataVisualizer.logger;

import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.BadLocationException;

import us.ihmc.robotDataLogger.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataLogger.logger.util.FileSelectionDialog;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoVariableExtracter
{
   public YoVariableExtracter(File logFile) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(logFile, YoVariableLoggerListener.propertyFile));
      File handshake = new File(logFile, logProperties.getHandshakeFile());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      YoVariableHandshakeParser parser = new YoVariableHandshakeParser("logged");
      parser.parseFrom(handshakeData);
      YoVariableRegistry registry = parser.getRootRegistry();

      File logdata = new File(logFile, logProperties.getVariableDataFile());
      if(!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }
      @SuppressWarnings("resource")
      final FileChannel logChannel = new FileInputStream(logdata).getChannel();

      List<YoVariable<?>> variables = parser.getYoVariablesList();
      int jointStateOffset = variables.size();
      int numberOfJointStates = JointState.getNumberOfJointStates(parser.getJointStates());
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;

      new YoVariableDialog(bufferSize, logChannel, variables, registry);
   }

   public static void main(String[] args) throws IOException
   {
            File file = FileSelectionDialog.loadDirectoryWithFileNamed("robotData.log");
            if(file != null)
            {
               new YoVariableExtracter(file);
            }

//      new YoVariableDialog(null);
   }

   public static class YoVariableDialog extends JFrame implements DocumentListener, ActionListener
   {
      private static final long serialVersionUID = 7786541677872539903L;
      private final YoVariableRegistry registry;
      private final JTextField searchField;
      private final JList<YoVariable<?>> result;
      private final List<YoVariable<?>> variables;
      
      private final FileChannel logChannel;
      private final int bufferSize;
      private final ByteBuffer logLine;
      private final LongBuffer logLongArray;

      public YoVariableDialog(int bufferSize, FileChannel logChannel, List<YoVariable<?>> variables, YoVariableRegistry registry)
      {
         super();
         setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
         setTitle("Select variable");
         setLocationRelativeTo(null);
         setLocationByPlatform(true);
         setSize(300, 300);
         this.bufferSize = bufferSize;
         this.registry = registry;
         this.logChannel = logChannel;
         this.variables = variables;
         
         searchField = new JTextField();
         result = new JList<YoVariable<?>>();
         result.setLayoutOrientation(JList.VERTICAL);
         result.setVisibleRowCount(10);
         JScrollPane scroller = new JScrollPane(result);
         scroller.setPreferredSize(new Dimension(300, 250));
         scroller.setAlignmentX(LEFT_ALIGNMENT);

         JButton print = new JButton("Export variable");
         print.addActionListener(this);

         Container container = getContentPane();
         container.setLayout(new BoxLayout(container, BoxLayout.PAGE_AXIS));
         container.add(searchField);
         container.add(scroller);
         container.add(print);
         setVisible(true);

         searchField.getDocument().addDocumentListener(this);
         
         
         logLine = ByteBuffer.allocate(bufferSize);
         logLongArray = logLine.asLongBuffer();
      }

      @Override
      public void insertUpdate(DocumentEvent e)
      {
         search(e);
      }

      @Override
      public void removeUpdate(DocumentEvent e)
      {
         search(e);
      }

      @Override
      public void changedUpdate(DocumentEvent e)
      {
         search(e);
      }

      private void search(DocumentEvent e)
      {
         String[] search = new String[1];
         String[] regexp = new String[2];
         try
         {
            search[0] = e.getDocument().getText(0, e.getDocument().getLength());
            regexp[0] = search[0];
            regexp[1] = search[0] + ".*";
         }
         catch (BadLocationException e1)
         {
            return;
         }
         
         ArrayList<YoVariable<?>> matchingVariables = registry.getMatchingVariables(search, regexp);
         YoVariable<?>[] variables = matchingVariables.toArray(new YoVariable[matchingVariables.size()]);
         
         result.setListData(variables);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         List<YoVariable<?>> results = result.getSelectedValuesList();
         
         printVariable(results);
      }

      private void printVariable(List<YoVariable<?>> variables)
      {
         int[] offsets = new int[variables.size()];
         StringBuffer result[] = new StringBuffer[variables.size()];
         
         for(int i = 0; i < variables.size(); i++)
         {
            YoVariable<?> variable = variables.get(i);
            result[i] = new StringBuffer();
            result[i].append(variable.getName());
            result[i].append(" = [");
            offsets[i] = this.variables.indexOf(variable);
         }
         ByteBuffer buffer = ByteBuffer.allocate(bufferSize);
         LongBuffer longBuffer = buffer.asLongBuffer();
         StringBuffer t = new StringBuffer();
         
         t.append("t = [");
         try
         {
            long tick = 0;
            
            logChannel.position(0);
            while(logChannel.size() > tick * bufferSize)
            {
               buffer.clear();
               logChannel.read(buffer);
               t.append(longBuffer.get(0));
               t.append(",");
               
               for(int i = 0; i < variables.size(); i++)
               {
                  YoVariable<?> variable = variables.get(i);
                  variable.setValueFromLongBits(longBuffer.get(1 + offsets[i]), false);
                  variable.getValueString(result[i]);
                  result[i].append(",");
               }
               
               tick++;
               if(tick % 10000 == 0)
               {
                  System.out.print(".");
                  System.out.flush();
               }
            }
            System.out.println();
            
            StringBuffer combine = new StringBuffer();
            t.deleteCharAt(t.length() - 1);
            t.append("];");
            t.append(System.lineSeparator());
            combine.append(t);
            for(int i = 0; i < variables.size(); i++)
            {
               result[i].deleteCharAt(result[i].length() - 1);
               result[i].append("];");
               result[i].append(System.lineSeparator());
               combine.append(result[i]);
            }
            Path path = Paths.get(variables.get(0).getName() + ".m");
            Files.write(path, combine.toString().getBytes());
            System.out.println("Wrote " + variables.get(0) + " to " + path);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         
      }
   }
}
