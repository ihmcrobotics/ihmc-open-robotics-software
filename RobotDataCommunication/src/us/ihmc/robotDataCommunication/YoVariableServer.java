package us.ihmc.robotDataCommunication;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotDataCommunication.jointState.JointHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class YoVariableServer implements RobotVisualizer
{
   private static final int VARIABLE_BUFFER_CAPACITY = 2048;
   private static final int CHANGED_BUFFER_CAPACITY = 64;
   
   private final double dt;
   
   // Variable data
   private final YoVariableRegistry registry;
   private final List<RigidBody> rootBodies = new ArrayList<RigidBody>();
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   
   private ConcurrentRingBuffer<FullStateBuffer> variableBuffer;
   private ConcurrentRingBuffer<VariableChangedMessage> changedMessageBuffer = new ConcurrentRingBuffer<VariableChangedMessage>(
         new VariableChangedMessage.Builder(), CHANGED_BUFFER_CAPACITY);
   
   private FullStateBuffer currentBuffer;
   private List<YoVariable> variables; 
   
   // State
   private boolean started = false;

   // ZMQ
   private final int handshakePort;
   private final int producerPort;
   private final int consumerPort;
   
   // Servers
   private YoVariableHandshakeServer handshakeServer;
   private YoVariableProducer producer;
   private YoVariableChangedConsumer consumer;

   
   public YoVariableServer(YoVariableRegistry registry, int port, double dt)
   {
      this(registry, port, dt, null, (RigidBody[]) null);
   }
   
   public YoVariableServer(YoVariableRegistry registry, int port, double dt, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, RigidBody... rootBodies)
   {
      this.handshakePort = port;
      this.producerPort = port + 1;
      this.consumerPort = port + 2;
      
      this.dt = dt;
      
      this.registry = registry;
      if(rootBodies != null)
      {
         this.rootBodies.addAll(Arrays.asList(rootBodies));
      }
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
   }

   public synchronized void start()
   {
      if(started)
      {
         throw new RuntimeException("Server already started");
      }
      
      handshakeServer = new YoVariableHandshakeServer(registry, handshakePort, rootBodies, dynamicGraphicObjectsListRegistry, dt);
      variables = handshakeServer.getVariables();
      
      List<JointHolder> jointHolders = handshakeServer.getJointHolders();
      FullStateBuffer.Builder builder = new FullStateBuffer.Builder(variables.size(), jointHolders);
      variableBuffer = new ConcurrentRingBuffer<FullStateBuffer>(builder, VARIABLE_BUFFER_CAPACITY);
      addVariableChangedListeners(variables);
      
      producer = new YoVariableProducer(producerPort, variables, jointHolders, variableBuffer);
      consumer = new YoVariableChangedConsumer(consumerPort, changedMessageBuffer);
      
      
      handshakeServer.start();
      producer.start();
      consumer.start();
      
      started = true;
   }

   private void addVariableChangedListeners(List<YoVariable> variables)
   {
      for(int i = 0; i < variables.size(); i++)
      {
         final int myID = i;
         variables.get(i).addVariableChangedListener(new VariableChangedListener()
         {
            public void variableChanged(YoVariable v)
            {
               if(currentBuffer != null)
               {
                  currentBuffer.update(myID, v.getValueAsLongBits());
               }
            }
         });
      }
   }

   public synchronized void close()
   {
      if(started)
      {
         handshakeServer.close();
         producer.close();
         consumer.close();         
      }
   }

   public void update(long timestamp)
   {
      if(!started)
      {
         return;
      }
      
      updateVariableBuffer(timestamp);
      updateChangedVariables();
   }

   private void updateChangedVariables()
   {
      changedMessageBuffer.poll();
      VariableChangedMessage msg;
      while((msg = changedMessageBuffer.read()) != null)
      {
         variables.get(msg.getId()).setValueFromDouble(msg.getVal());            
      }
      changedMessageBuffer.flush();
   }

   private void updateVariableBuffer(long timestamp)
   {
      if(currentBuffer != null)
      {
         currentBuffer.updateJointStates(timestamp);
      }
      variableBuffer.commit();
      currentBuffer = variableBuffer.next();
   }

   

   public void setFullRobotModel(FullRobotModel controllerModel)
   {
      rootBodies.add(controllerModel.getElevator());
   }

}
