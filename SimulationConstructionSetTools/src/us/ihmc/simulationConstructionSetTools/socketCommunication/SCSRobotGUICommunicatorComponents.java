package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;

import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;

public class SCSRobotGUICommunicatorComponents implements ExitActionListener, DoDisconnectListener
{
   private ConnectButton connectButton;
   private PauseButton pauseButton;
   private final boolean debug = false;

   // private RecordCheckBox recordCheckBox;
   private JCheckBox recordCheckBox;
   private final RobotConnection connection;

   @Override
   public void doDisconnect()
   {
      if (connectButton != null)
      {
         connectButton.setText("Connect");
         recordCheckBox.setSelected(false);
         connection.setRecord(false);
      }
   }

   public SCSRobotGUICommunicatorComponents(RobotConnection connection)
   {
      this.connection = connection;
   }

   public void putButtonsAndExitActionListenerOnSimulationGUI(final SimulationConstructionSet simulationConstructionSet)
   {
      // Add some buttons:
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            connectButton = new ConnectButton(connection);
            connectButton.setRequestFocusEnabled(false);
            simulationConstructionSet.addButton(connectButton);

            pauseButton = new PauseButton();
            pauseButton.setRequestFocusEnabled(false);
            if (debug)
               simulationConstructionSet.addButton(pauseButton);

            recordCheckBox = new RecordCheckBox();
            recordCheckBox.setRequestFocusEnabled(false);
            recordCheckBox.setSelected(false);
            simulationConstructionSet.addCheckBox(recordCheckBox);
         }
      });

      simulationConstructionSet.attachExitActionListener(this);
   }

   @Override
   public void exitActionPerformed()
   {
      System.out.println("Disconnecting before exiting...");
      connection.disconnect();
   }

   public class PauseButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 3465225805490022043L;

      public PauseButton()
      {
         super("Pause");
         addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent evt)
      {
         connection.pause();
      }

   }

   public class RecordCheckBox extends JCheckBox implements ActionListener
   {
      private static final long serialVersionUID = 3858416227113728288L;

      public RecordCheckBox()
      {
         super("Record");
         addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent evt)
      {
         connection.setRecord(this.isSelected());
      }
   }

   public class ConnectButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -2191752688240579800L;

      //private final RobotConnection connection;

      // private final RobotConnection connection;

      public ConnectButton(RobotConnection connection)
      {
         super("Connect");

         // this.connection = connection;
         addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent evt)
      {

         connection.attemptConnectionToHost();

         if (connection.isConnected())
         {
            setText("Disconnect");
            recordCheckBox.setSelected(true);
            connection.setRecord(true);
         }
         else
         {
            setText("Connect");
            recordCheckBox.setSelected(false);
            connection.setRecord(false);
         }
      }
   }

}
