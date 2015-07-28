package us.ihmc.darpaRoboticsChallenge;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.Properties;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelShell;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class DRCFilteredLogPlaybackSelector
{
   public static final int CONNECTION_TIMEOUT = 30000;
   private JFrame frame;

   JSch jsch;
   private Properties config = new Properties();
   private PrintStream shellPrintStream;
   public static final String PLAYBACK_LOG_COMMAND = "gazebo -s libgazebo_ros_api_plugin.so -p filtered_state.log";
   private Session session;

   private String[] tasks = new String[] {"Driving", "Walking", "Hose"};
   private Integer[] runs = new Integer[] {1, 2, 3, 4, 5};

   public DRCFilteredLogPlaybackSelector()
   {
      try
      {
         initShellConnection();
      }
      catch (JSchException e)
      {
         // do nothing
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void initShellConnection() throws JSchException, IOException
   {
      jsch = new JSch();

      config = new Properties();
      config.put("StrictHostKeyChecking", "no");

      session = jsch.getSession("unknownid", "127.0.0.1", 22);

      session.setConfig(config);

      session.setPassword("unknownpw");

      session.connect(CONNECTION_TIMEOUT);

      ChannelShell channel = (ChannelShell) session.openChannel("shell");
      shellPrintStream = new PrintStream(channel.getOutputStream(), true);

      channel.setOutputStream(null);
      channel.setInputStream(null);

      channel.connect(CONNECTION_TIMEOUT);

      channel.setOutputStream(System.out);

      checkForRosCore();
   }

   private void initSwingGui()
   {
      try
      {
         UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
      }
      catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException e1)
      {
         e1.printStackTrace();
      }

      frame = new JFrame("Filtered Log Playback Selector");
      frame.setSize(350, 100);
      frame.setResizable(false);
      frame.setLocationRelativeTo(null);

      JPanel panel = new JPanel();

      panel.add(new JLabel("Task: ", JLabel.CENTER));
      final JComboBox taskCombo = new JComboBox(tasks);
      panel.add(taskCombo);

      panel.add(new JLabel("Run Number: ", JLabel.CENTER));
      final JComboBox runCombo = new JComboBox(runs);
      panel.add(runCombo);

      JButton launchButton = new JButton("Start Playback");
      launchButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            int taskNumber = taskCombo.getSelectedIndex() + 1;
            int runNumber = runCombo.getSelectedIndex() + 1;

            launchLogPlayback(taskNumber, runNumber);
         }
      });

      panel.add(launchButton);

      frame.getContentPane().add(panel);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setVisible(true);
   }

   private void checkForRosCore() throws JSchException, IOException
   {
      int pid = -1;
      ChannelExec channel = (ChannelExec) session.openChannel("exec");
      channel.setCommand("ps aux | grep roscore | grep -v grep | awk '{ print $2 }'");
      channel.setInputStream(null);
      channel.setOutputStream(System.out);

      BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

      channel.connect(CONNECTION_TIMEOUT);

      String tmp = reader.readLine();

      if (tmp != null)
         pid = Integer.parseInt(tmp);

      channel.disconnect();

      if (pid < 0)
      {
         shellPrintStream.println("roscore &");
      }
   }

   private void launchLogPlayback(int taskNumber, int runNumber)
   {
      validateTaskAndRunNumbers(taskNumber, runNumber);

      changeToDirectory(taskNumber, runNumber);

      beginPlayback();
   }

   private void changeToDirectory(int taskNumber, int runNumber)
   {
      String dirName = "vrc_final_" + taskNumber + "_" + runNumber;

      shellPrintStream.println("cd ~/VRC\\ Final\\ Logs/" + dirName);
      shellPrintStream.flush();
   }

   private void beginPlayback()
   {
      shellPrintStream.println(PLAYBACK_LOG_COMMAND);
      shellPrintStream.flush();
   }

   private void validateTaskAndRunNumbers(int taskNumber, int runNumber)
   {
      if ((taskNumber <= 0) || (taskNumber > 3))
         throw new RuntimeException("Invalid Task Number");
      if ((runNumber <= 0) || (runNumber > 5))
         throw new RuntimeException("Invalid Run Number");
   }


   public static void main(String[] args)
   {
      final DRCFilteredLogPlaybackSelector drcFilterdLogPlaybackSelector = new DRCFilteredLogPlaybackSelector();

      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            drcFilterdLogPlaybackSelector.initSwingGui();
         }
      });
   }
}
