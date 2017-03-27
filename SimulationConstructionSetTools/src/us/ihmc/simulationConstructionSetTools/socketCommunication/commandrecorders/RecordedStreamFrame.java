package us.ihmc.simulationConstructionSetTools.socketCommunication.commandrecorders;

import java.awt.Container;

import javax.swing.JFrame;
import javax.swing.JPanel;


public class RecordedStreamFrame
{
   private Container contentPane;
   private RecordedStreamPanel recordedStreamPanel;

   public RecordedStreamFrame()
   {
      JFrame frame = new JFrame("Recorded Stream");

      contentPane = frame.getContentPane();

      recordedStreamPanel = new RecordedStreamPanel();
      contentPane.add(recordedStreamPanel);


      frame.pack();
      frame.setVisible(true);
   }

   private class RecordedStreamPanel extends JPanel
   {

      /**
       *
       */
      private static final long serialVersionUID = 3589257111826282041L;
   }

}
