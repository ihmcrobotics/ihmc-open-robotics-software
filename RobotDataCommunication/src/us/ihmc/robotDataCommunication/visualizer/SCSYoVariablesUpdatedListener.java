package us.ihmc.robotDataCommunication.visualizer;


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

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class SCSYoVariablesUpdatedListener implements YoVariablesUpdatedListener
{
   protected final YoVariableRegistry registry;
   protected final SimulationConstructionSet scs;

   private final Robot robot;
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<JointUpdater>();
   private boolean recording = true;

   public SCSYoVariablesUpdatedListener(int bufferSize)
   {
      this(new Robot("NullRobot"),bufferSize);
   }

   public SCSYoVariablesUpdatedListener(Robot robot, int bufferSize)
   {
      this.robot = robot;
      this.scs = new SimulationConstructionSet(robot,bufferSize);
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
   }



   public void start()
   {
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

         scs.tickAndUpdate();
      }
   }

   public void setJointStates(List<JointState<? extends Joint>> jointStates)
   {
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);
   }



   public void registerDynamicGraphicObjectListsRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);



      VisualizerUtils.createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs);
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
}
