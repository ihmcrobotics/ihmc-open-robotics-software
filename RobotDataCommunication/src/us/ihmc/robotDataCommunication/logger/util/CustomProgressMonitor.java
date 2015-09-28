package us.ihmc.robotDataCommunication.logger.util;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.Insets;
import java.awt.Window;
import java.io.OutputStream;
import java.io.PrintWriter;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import us.ihmc.tools.gui.TextAreaOutputStream;
import us.ihmc.tools.thread.ThreadTools;


public class CustomProgressMonitor extends JDialog
{
   private static final long serialVersionUID = 2256156304298596266L;
   private final JProgressBar progressBar;
   private final JTextArea commandOutput;
   private final JLabel noteLabel;
   
   private final TextAreaOutputStream outputStream ;
   
   public CustomProgressMonitor(String message, String note, int min, int max)
   {
      super((Window) null, "Progress");
      setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
      
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
   
   public void setNote(String note)
   {
      noteLabel.setText(note);
   }

   public void setProgress(int i)
   {
      progressBar.setValue(i);
   }
   
   public OutputStream getOutputStream()
   {
      return outputStream;
   }
   
   public void close()
   {
      setVisible(false);
      dispose();
   }
   
   public static void main(String[] args)
   {
      CustomProgressMonitor monitor = new CustomProgressMonitor("test", "testNote", 0, 100);
      PrintWriter writer = new PrintWriter(monitor.getOutputStream(), true);
      for(int i = 0; i < 100; i++)
      {
         monitor.setProgress(i);
         monitor.setNote("Progress: " + i);
         writer.println(i);
         ThreadTools.sleep(100);
      }
      
      monitor.close();

   }

   public void setError(String string)
   {
      noteLabel.setText(string);
      noteLabel.setForeground(Color.red);
      Font font = noteLabel.getFont();
      noteLabel.setFont(new Font(font.getFontName(), Font.BOLD, font.getSize()));
      
   }

}
