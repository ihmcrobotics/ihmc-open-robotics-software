package us.ihmc.robotDataCommunication.logger;

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

import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotDataCommunication.logger.util.FileSelectionDialog;
import us.ihmc.utilities.math.TimeTools;

import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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

      YoVariableHandshakeParser parser = new YoVariableHandshakeParser(null, "logged", true);
      parser.parseFrom(handshakeData);

      YoVariableRegistry registry = parser.getYoVariableRegistry();

      File logdata = new File(logFile, logProperties.getVariableDataFile());
      if(!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }
      @SuppressWarnings("resource")
      final FileChannel logChannel = new FileInputStream(logdata).getChannel();

      List<YoVariable> variables = parser.getYoVariablesList();
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
      private final YoVariableRegistry registry;
      private final JTextField searchField;
      private final JList<YoVariable> result;
      private final List<YoVariable> variables;
      
      private final FileChannel logChannel;
      private final int bufferSize;
      private final ByteBuffer logLine;
      private final LongBuffer logLongArray;

      public YoVariableDialog(int bufferSize, FileChannel logChannel, List<YoVariable> variables, YoVariableRegistry registry)
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
         result = new JList<YoVariable>();
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
         
         ArrayList<YoVariable> matchingVariables = registry.getMatchingVariables(search, regexp);
         YoVariable[] variables = matchingVariables.toArray(new YoVariable[matchingVariables.size()]);
         
         
         result.setListData(variables);
         
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         List<YoVariable> results = result.getSelectedValuesList();
         
         for(YoVariable variable : results)
         {
            printVariable(variable);
         }
      }

      private void printVariable(YoVariable variable)
      {
         int offset = variables.indexOf(variable);
         ByteBuffer buffer = ByteBuffer.allocate(8);
         LongBuffer longBuffer = buffer.asLongBuffer();
         StringBuffer t = new StringBuffer();
         StringBuffer result = new StringBuffer();
         result.append(variable.getName());
         result.append(" = [");
         
         t.append("t = [");
         try
         {
            long tick = 0;
            
            while(logChannel.size() > tick * bufferSize)
            {
               logChannel.position(tick * bufferSize);
               buffer.clear();
               logChannel.read(buffer);
               t.append(longBuffer.get(0));
               t.append(",");
               
               logChannel.position(tick * bufferSize + 8 * (1 + offset));
               buffer.clear();
               logChannel.read(buffer);
               
               variable.setValueFromLongBits(longBuffer.get(0), false);
               variable.getValueString(result);
               result.append(",");
               
               tick++;
               if(tick % 10000 == 0)
               {
                  System.out.print(".");
                  System.out.flush();
               }
            }
            System.out.println();
            result.deleteCharAt(result.length() - 1);
            t.deleteCharAt(t.length() - 1);
            result.append("];");
            result.append(System.lineSeparator());
            t.append("];");
            t.append(System.lineSeparator());
            t.append(result);
            Path path = Paths.get(variable.getName() + ".m");
            Files.write(path, t.toString().getBytes());
            System.out.println("Wrote " + variable.getName() + " to " + path);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         
      }
   }
}
