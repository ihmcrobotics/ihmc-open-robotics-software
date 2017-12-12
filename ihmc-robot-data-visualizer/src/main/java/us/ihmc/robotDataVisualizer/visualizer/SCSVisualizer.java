package us.ihmc.robotDataVisualizer.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Main entry point for the visualizer. 
 * 
 * To make a custom visualizer for your robot, do NOT copy, instead extend. 
 *  
 * @author jesper
 *
 */
public class SCSVisualizer implements YoVariablesUpdatedListener, ExitActionListener, SCSVisualizerStateListener
{
   private final static String DISCONNECT_DISCONNECT = "Disconnect";
   private final static String DISCONNECT_RECONNECT = "Reconnect";
   
   
   private YoVariableRegistry registry;
   private SimulationConstructionSet scs;

   
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<>();
   private YoVariableClientInterface yoVariableClientInterface;
   private ArrayList<SCSVisualizerStateListener> stateListeners = new ArrayList<>();

   private int displayOneInNPackets = 1;
   private long counter = 0;

   private final TObjectDoubleHashMap<String> buttons = new TObjectDoubleHashMap<>();

   private final Object disconnectLock = new Object();
   private final JButton disconnectButton = new JButton("Waiting for connection...");
   private final JButton clearLogButton = new JButton("Clear log");

   private final DecimalFormat delayFormat = new DecimalFormat("0000");
   private final JLabel delayValue = new JLabel();


   private volatile long lastTimestamp;

   private int bufferSize;
   private boolean showGUI;
   private boolean hideViewport;
   private boolean showOverheadView;

   private YoGraphicsListRegistry yoGraphicsListRegistry;

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
      this.disconnectButton.setEnabled(false);
      addSCSVisualizerStateListener(this);
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      if(++counter % displayOneInNPackets == 0)
      {
         long delay = Conversions.nanosecondsToMilliseconds(lastTimestamp - timestamp);
         delayValue.setText(delayFormat.format(delay));
   
         for (int i = 0; i < jointUpdaters.size(); i++)
         {
            jointUpdaters.get(i).update();
         }
         scs.setTime(Conversions.nanosecondsToSeconds(timestamp));
         updateLocalVariables();
         scs.tickAndUpdate();
      }
   }

   @Override
   public void disconnected()
   {
      synchronized(disconnectLock)
      {
         System.out.println("Disconnected. Sliders now enabled.");
         disconnectButton.setText(DISCONNECT_RECONNECT);
         disconnectButton.setEnabled(true);
         scs.setScrollGraphsEnabled(true);
      }
   }

   @Override
   public void connected()
   {
      synchronized(disconnectLock)
      {
         scs.setInPoint();
         disconnectButton.setText(DISCONNECT_DISCONNECT);
         disconnectButton.setEnabled(true);
      }
   }

   public void addButton(String yoVariableName, double newValue)
   {
      buttons.put(yoVariableName, newValue);
   }

   @Override
   public boolean changesVariables()
   {
      return true;
   }


   @Override
   public boolean updateYoVariables()
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

   public void addButtonToSimulationConstructionSetGUI(AbstractButton button)
   {
      scs.addButton(button);
   }

   @Override
   public void exitActionPerformed()
   {
      if (yoVariableClientInterface != null)
      {
         yoVariableClientInterface.stop();
      }
   }

   @Override
   public int getDisplayOneInNPackets()
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
   public void setShowOverheadView(boolean showOverheadView)
   {
      this.showOverheadView = showOverheadView;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   @Override
   public final void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      this.yoVariableClientInterface = yoVariableClientInterface;
      
      Robot robot = new Robot("DummyRobot");
      if (handshake.getModelLoaderClass() != null)
      {
         LogModelLoader modelLoader;
         try
         {
            modelLoader = (LogModelLoader) Class.forName(handshake.getModelLoaderClass()).newInstance();
         }
         catch (Exception e)
         {
            System.err.println("Could not instantiate LogModelLoader: " + handshake.getModelLoaderClass() + ". Defaulting to SDFModelLoader.");
            modelLoader = new SDFModelLoader();
         }
         modelLoader.load(handshake.getModelName(), handshake.getModel(), handshake.getResourceDirectories(), handshake.getResourceZip(), null);
         robot = new RobotFromDescription(modelLoader.createRobot());
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(this.bufferSize);

      this.scs = new SimulationConstructionSet(robot, parameters);
      if (hideViewport) { scs.hideViewport(); }
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
      scs.attachExitActionListener(this);
      scs.attachPlaybackListener(createYoGraphicsUpdater());
      scs.setRunName(yoVariableClientInterface.getServerName());
      //scs.setFastSimulate(true, 50);

      scs.addButton(disconnectButton);
      disconnectButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            synchronized (disconnectLock)
            {
               if (disconnectButton.getText().equals(DISCONNECT_DISCONNECT))
               {
                  disconnectButton.setEnabled(false);
                  yoVariableClientInterface.disconnect();
               }
               else
               {
                  try
                  {
                     System.out.println("Reconnecting. Disabling sliders.");
                     scs.gotoOutPointNow();
                     scs.setScrollGraphsEnabled(false);
                     scs.tick(0);

                     if (yoVariableClientInterface.reconnect())
                     {
                        disconnectButton.setEnabled(false);
                     }
                     else
                     {
                        JOptionPane.showMessageDialog(scs.getJFrame(), "Cannot reconnect. No matching sessions found.", "Cannot reconnect",
                                                      JOptionPane.ERROR_MESSAGE);
                        System.out.println("Reconnect failed. Enabling sliders.");
                        scs.setScrollGraphsEnabled(true);

                     }
                  }
                  catch (IOException reconnectError)
                  {
                     reconnectError.printStackTrace();
                  }
               }
            }
         }

      });
      
      scs.addButton(clearLogButton);
      clearLogButton.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (yoVariableClientInterface != null)
            {
               yoVariableClientInterface.sendClearLogRequest();
            }
         }
      });

      scs.addJLabel(new JLabel("Delay: "));
      scs.addJLabel(delayValue);
      scs.addJLabel(new JLabel("ms"));

      YoVariableRegistry yoVariableRegistry = handshakeParser.getRootRegistry();
      this.registry.addChild(yoVariableRegistry);
      this.registry.addChild(yoVariableClientInterface.getDebugRegistry());
      scs.setParameterRootPath(yoVariableRegistry);
      
      List<JointState> jointStates = handshakeParser.getJointStates();
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);

      yoGraphicsListRegistry = handshakeParser.getYoGraphicsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      for (int i = 0; i < stateListeners.size(); i++)
      {
         SCSVisualizerStateListener stateListener = stateListeners.get(i);
         stateListener.starting(scs, robot, this.registry);
      }



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

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
   }
   
   /**
    * Overridable method to update local variables.
    * 
    * Needs to return in less than your visualization dt.
    */
   public void updateLocalVariables()
   {
      
   }

   public static void main(String[] args)
   {
      SCSVisualizer visualizer = new SCSVisualizer(32169, true);
      visualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(visualizer);
      client.start();
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      lastTimestamp = timestamp;
   }

   @Override
   public void clearLog(String guid)
   {
   }

   private PlaybackListener createYoGraphicsUpdater()
   {
      return new PlaybackListener()
      {
         @Override
         public void notifyOfIndexChange(int newIndex)
         {
            updateYoGraphics();
         }

         @Override
         public void stop()
         {
         }

         @Override
         public void play(double realTimeRate)
         {
         }
      };
   }

   private void updateYoGraphics()
   {
      if (yoGraphicsListRegistry == null)
         return;

      List<YoGraphicsList> yoGraphicsLists = yoGraphicsListRegistry.getYoGraphicsLists();
      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
         for (YoGraphic yoGraphic : yoGraphics)
            yoGraphic.update();
      }
      yoGraphicsListRegistry.update();
   }
}
