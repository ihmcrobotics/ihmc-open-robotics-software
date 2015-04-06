package us.ihmc.robotDataCommunication.visualizer;

import gnu.trove.map.hash.TObjectDoubleHashMap;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;

import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class SCSVisualizer implements YoVariablesUpdatedListener, ExitActionListener, SCSVisualizerStateListener
{
   private static final int DISPLAY_ONE_IN_N_PACKETS = 6;

   protected YoVariableRegistry registry;
   protected SimulationConstructionSet scs;

   private Robot robot;
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<JointUpdater>();
   private volatile boolean recording = true;
   private YoVariableClient client;
   private ArrayList<SCSVisualizerStateListener> stateListeners = new ArrayList<>();

   private int displayOneInNPackets = DISPLAY_ONE_IN_N_PACKETS;

   private final TObjectDoubleHashMap<String> buttons = new TObjectDoubleHashMap<String>();

   private final JButton disconnectButton = new JButton("Disconnect");
   private final JButton clearLogButton = new JButton("Clear log");

   private int bufferSize;
   private boolean showGUI;
   private boolean hideViewport;

   public SCSVisualizer(int bufferSize)
   {
      this(bufferSize, true);
   }

   public SCSVisualizer(int bufferSize, boolean showGUI)
   {
      this(bufferSize, showGUI, false);
   }

   public SCSVisualizer(int bufferSize, boolean showGUI, boolean hideViewport)
   {
      this.bufferSize = bufferSize;
      this.showGUI = showGUI;
      this.hideViewport = hideViewport;
      addSCSVisualizerStateListener(this);
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

   public void disconnected()
   {
      System.out.println("DISCONNECTED. SLIDERS NOW ENABLED");
      scs.setScrollGraphsEnabled(true);
   }

   public void setYoVariableClient(final YoVariableClient client)
   {

      this.client = client;
   }

   private void disconnect(final JButton disconnectButton)
   {
      disconnectButton.setEnabled(false);
      client.requestStop();
   }

   public void addButton(String yoVariableName, double newValue)
   {
      buttons.put(yoVariableName, newValue);
   }

   public void receivedHandshake(LogHandshake handshake)
   {
      // Ignore
   }

   public boolean changesVariables()
   {
      return true;
   }

   public void receiveTimedOut()
   {
      System.out.println("Connection lost, closing client.");
      client.disconnected();

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
         client.requestStop();
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

   public void updateGraphsLessFrequently(boolean enable, int numberOfTicksBeforeUpdatingGraphs)
   {
      scs.setFastSimulate(enable, numberOfTicksBeforeUpdatingGraphs);
   }

   public void addSCSVisualizerStateListener(SCSVisualizerStateListener stateListener)
   {
      stateListeners.add(stateListener);
   }

   @Override
   public final void start(LogModelLoader logModelLoader, YoVariableRegistry yoVariableRegistry, List<JointState<? extends Joint>> jointStates,
         YoGraphicsListRegistry yoGraphicsListRegistry, int bufferSize, boolean showOverheadView)
   {

      if (logModelLoader == null)
      {
         this.robot = new Robot("DummyRobot");
      }
      else
      {
         this.robot = logModelLoader.createRobot();
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(this.bufferSize);
      this.scs = new SimulationConstructionSet(robot, parameters);
      if (hideViewport)
         scs.hideViewport();
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
      scs.attachExitActionListener(this);
      scs.setFastSimulate(true, 50);

      scs.addButton(disconnectButton);
      disconnectButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            disconnect(disconnectButton);
         }

      });
      scs.addButton(clearLogButton);
      clearLogButton.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (client != null)
            {
               client.sendClearLogRequest();
            }
         }
      });

      this.registry.addChild(yoVariableRegistry);
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);

      for (SCSVisualizerStateListener stateListener : stateListeners)
         stateListener.starting(scs, robot, this.registry);

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

   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
   }

   public static void main(String[] args)
   {
      YoVariableClient client = new YoVariableClient(new SCSVisualizer(32169, true), "remote", false);
      client.start();
   }

   @Override
   public void timestampReceived(long timestamp)
   {
   }

   @Override
   public void clearLog()
   {
   }
}
