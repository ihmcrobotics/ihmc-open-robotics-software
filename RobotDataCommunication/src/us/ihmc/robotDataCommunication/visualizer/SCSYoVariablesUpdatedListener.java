package us.ihmc.robotDataCommunication.visualizer;

import gnu.trove.map.hash.TObjectDoubleHashMap;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;

import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.ExitActionListener;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class SCSYoVariablesUpdatedListener implements YoVariablesUpdatedListener, ExitActionListener
{
   private static final int DISPLAY_ONE_IN_N_PACKETS = 6;

   protected final YoVariableRegistry registry;
   protected final SimulationConstructionSet scs;

   private final Robot robot;
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<JointUpdater>();
   private volatile boolean recording = true;
   private YoVariableClient client;

   private int displayOneInNPackets = DISPLAY_ONE_IN_N_PACKETS;

   private final TObjectDoubleHashMap<String> buttons = new TObjectDoubleHashMap<String>();

   public SCSYoVariablesUpdatedListener(int bufferSize)
   {
      this(bufferSize, true);
   }

   public SCSYoVariablesUpdatedListener(int bufferSize, boolean showGUI)
   {
      this(new Robot("NullRobot"), bufferSize, showGUI);
   }
   
   public SCSYoVariablesUpdatedListener(Robot robot, int bufferSize)
   {
      this(robot, bufferSize, true);
   }
   
   public SCSYoVariablesUpdatedListener(Robot robot, int bufferSize, boolean showGUI)
   {
      this.robot = robot;
      this.scs = new SimulationConstructionSet(robot, showGUI, bufferSize);
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
      scs.attachExitActionListener(this);
   }


   public void start()
   {
      for (String yoVariableName : buttons.keySet())
      {
         final YoVariable<?> var = registry.getVariable(yoVariableName);
         final JButton button = new JButton(yoVariableName);
         final double newValue = buttons.get(yoVariableName);
         scs.addButton(button);
         button.addActionListener(new ActionListener()
         {

            @Override
            public void actionPerformed(ActionEvent e)
            {
               var.setValueFromDouble(newValue);
            }
         });
      }

      new Thread(scs).start();
   }

   public void setRegistry(YoVariableRegistry registry)
   {
      this.registry.addChild(registry);
   }

   public void receivedUpdate(long timestamp, ByteBuffer buffer)
   {
      if (recording)
      {
         for (int i = 0; i < jointUpdaters.size(); i++)
         {
            jointUpdaters.get(i).update();
         }
         scs.setTime(TimeTools.nanoSecondstoSeconds(timestamp));
         scs.tickAndUpdate();
      }
   }

   public void setJointStates(List<JointState<? extends Joint>> jointStates)
   {
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);
   }

   public void registerDynamicGraphicObjectListsRegistry(YoGraphicsListRegistry yoGraphicsListRegistry, boolean showOverheadView)
   {
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);
   }

   public void disconnected()
   {
      System.out.println("DISCONNECTED. SLIDERS NOW ENABLED");
      scs.setScrollGraphsEnabled(true);
   }

   public void setYoVariableClient(final YoVariableClient client)
   {
      final JButton disconnectButton = new JButton("disconnect");
      scs.addButton(disconnectButton);
      disconnectButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            disconnectButton.setEnabled(false);
            client.close();
         }
      });

      this.client = client;
   }

   public void addButton(String yoVariableName, double newValue)
   {
      buttons.put(yoVariableName, newValue);
   }

   public void receivedHandshake(YoProtoHandshake handshake)
   {
      // Ignore
   }

   public boolean changesVariables()
   {
      return true;
   }

   public void receiveTimedOut(long timeoutInMillis)
   {
      // TODO Auto-generated method stub
   }

   public boolean populateRegistry()
   {
      return true;
   }

   public void closeAndDispose()
   {
      scs.closeAndDispose();
   }

   public DataBuffer getDataBuffer()
   {
      return scs.getDataBuffer();
   }

   @Override
   public void exitActionPerformed()
   {
      recording = false;
      if (client != null)
      {
         client.close();
      }
   }

   @Override
   public long getDisplayOneInNPackets()
   {
      return displayOneInNPackets;
   }

   public void setDisplayOneInNPackets(int val)
   {
      displayOneInNPackets = val;
   }
}
