package us.ihmc.avatar.sensors.microphone;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * Sound Detector Gui
 * 
 * Press start to listen to microphone input and run the rotozip sound detector. "Rotozip ON!!!!" should show up on the gui when the sound detector notices it.
 * 
 * @author Will
 *
 */

public class SoundDetectorGui implements ActionListener
{
   private static SoundDetector soundDetector;
   JPanel indicationPanel, buttonPanel;
   static JLabel statusIndicator;
   JButton startStopButton;
   private boolean isRunning = false;
   private static Thread soundDetectionThread;

   public JPanel createContentPane()
   {

      JPanel soundDetectorGUI = new JPanel();
      soundDetectorGUI.setLayout(null);

      indicationPanel = new JPanel();
      indicationPanel.setLayout(null);
      indicationPanel.setLocation(10, 0);
      indicationPanel.setSize(360, 30);
      soundDetectorGUI.add(indicationPanel);

      buttonPanel = new JPanel();
      buttonPanel.setLayout(null);
      buttonPanel.setLocation(10, 80);
      buttonPanel.setSize(360, 70);
      soundDetectorGUI.add(buttonPanel);

      startStopButton = new JButton("Start");
      startStopButton.setLocation(0, 0);
      startStopButton.setSize(300, 30);
      startStopButton.addActionListener(this);
      buttonPanel.add(startStopButton);

      statusIndicator = new JLabel("Press Start");
      statusIndicator.setLocation(0, 0);
      statusIndicator.setSize(300, 30);
      indicationPanel.add(statusIndicator);

      soundDetectorGUI.setOpaque(true);
      return soundDetectorGUI;
   }

   public void actionPerformed(ActionEvent e)
   {
      if (e.getSource() == startStopButton)
      {
         //Will is new to threading. Please excuse his sloppy hacks.
         if (!isRunning)
         {
            isRunning = true;
            startStopButton.setText("Stop");
            soundDetectionThread = new Thread(soundDetector);
            soundDetectionThread.start();
            statusIndicator.setText("Listening...");
         }
         else
         {
            isRunning = false;
            startStopButton.setText("Start");
            soundDetectionThread.interrupt();
            soundDetector.stop();
            statusIndicator.setText("Press Start");
         }
      }
   }

   private static void createAndShowGUI()
   {
      JFrame.setDefaultLookAndFeelDecorated(true);
      JFrame frame = new JFrame("Sound Detection");
      SoundDetectorGui demo = new SoundDetectorGui();
      frame.setContentPane(demo.createContentPane());
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setSize(360, 190);
      frame.setVisible(true);
      soundDetector = new SoundDetector(statusIndicator);
   }

   public static void main(String[] args)
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            createAndShowGUI();
         }
      });
   }
}