package us.ihmc.robotDataLogger.logger.util;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.Insets;
import java.awt.Window;
import java.io.OutputStream;
import java.io.PrintStream;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import org.apache.commons.io.output.TeeOutputStream;

import us.ihmc.tools.gui.TextAreaOutputStream;
import us.ihmc.tools.thread.ThreadTools;


public class CustomProgressMonitor extends JDialog implements ProgressMonitorInterface
{
   private static final long serialVersionUID = 2256156304298596266L;
   private JProgressBar progressBar;
   private JTextArea commandOutput;
   private JLabel noteLabel;
   
   private TextAreaOutputStream outputStream ;
   private PrintStream printStream;
   
   private boolean echoToConsole = false;
   
   public CustomProgressMonitor()
   {
      super((Window) null, "Progress");
      setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);  
   }
   
   public CustomProgressMonitor(String message, String note, int min, int max)
   {
      this();
      initialize(message, note, min, max);
   }
   
   public void setEchoToConsole(boolean echoToConsole)
   {
      if(printStream != null)
      {
         throw new RuntimeException("Call setEchoToConsole before initialize()");
      }
      this.echoToConsole = echoToConsole;
   }
   
   @Override
   public void initialize(String message, String note, int min, int max)
   {
      JPanel mainPanel = new JPanel(new BorderLayout());
      mainPanel.setOpaque(true);;
      
      if (note != null)
      {
         noteLabel = new JLabel(note);
      }
      else
      {
         noteLabel = null;
      }

      progressBar = new JProgressBar(min, max);
      progressBar.setValue(min);
      progressBar.setStringPainted(true);

      commandOutput = new JTextArea(5, 20);
      commandOutput.setMargin(new Insets(5, 5, 5, 5));
      commandOutput.setEditable(false);
      outputStream = new TextAreaOutputStream(commandOutput);
      
      OutputStream stream;
      if(echoToConsole)
      {
         stream = new TeeOutputStream(outputStream, System.out);
      }
      else
      {
         stream = outputStream;
      }
      printStream = new PrintStream(stream, true);

      JPanel panel = new JPanel();
      panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
      if (message != null)
      {
         panel.add(new JLabel(message));
      }
      if (noteLabel != null)
      {
         panel.add(noteLabel);
      }
      panel.add(progressBar);

      mainPanel.add(panel, BorderLayout.PAGE_START);
      mainPanel.add(new JScrollPane(commandOutput), BorderLayout.CENTER);
      mainPanel.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));
      
      setContentPane(mainPanel);
      pack();
      setSize(800, getHeight());
      setLocationByPlatform(true);
      setVisible(true);

   }
   
   @Override
   public void setNote(String note)
   {
      noteLabel.setText(note);
   }

   @Override
   public void setProgress(int i)
   {
      progressBar.setValue(i);
   }
   
   @Override
   public PrintStream getPrintStream()
   {
      return printStream;
   }
   
   @Override
   public void close()
   {
      setVisible(false);
      dispose();
   }
   
   public static void main(String[] args)
   {
      ProgressMonitorInterface monitor = new CustomProgressMonitor("test", "testNote", 0, 100);
      PrintStream writer = monitor.getPrintStream();
      for(int i = 0; i < 100; i++)
      {
         monitor.setProgress(i);
         monitor.setNote("Progress: " + i);
         writer.println(i);
         ThreadTools.sleep(100);
      }
      
      monitor.close();

   }

   @Override
   public void setError(String string)
   {
      noteLabel.setText(string);
      noteLabel.setForeground(Color.red);
      Font font = noteLabel.getFont();
      noteLabel.setFont(new Font(font.getFontName(), Font.BOLD, font.getSize()));
      
   }

}
