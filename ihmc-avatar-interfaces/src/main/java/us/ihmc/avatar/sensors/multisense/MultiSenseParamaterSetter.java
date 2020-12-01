package us.ihmc.avatar.sensors.multisense;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;

import org.ros.exception.RemoteException;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseListener;

import controller_msgs.msg.dds.MultisenseParameterPacket;
import dynamic_reconfigure.BoolParameter;
import dynamic_reconfigure.DoubleParameter;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;
import dynamic_reconfigure.StrParameter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.processManagement.ProcessStreamGobbler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.publisher.RosDoublePublisher;

public class MultiSenseParamaterSetter implements PacketConsumer<MultisenseParameterPacket>
{
   private double gain;
   private double motorSpeed;
   private boolean ledEnable;
   private boolean flashEnable;
   private double dutyCycle;
   private boolean autoExposure;
   private boolean autoWhitebalance;
   private final RosServiceClient<ReconfigureRequest, ReconfigureResponse> multiSenseClient;
   private final RosMainNode rosMainNode;
   private ParameterTree params;
   private IHMCROS2Publisher<MultisenseParameterPacket> publisher;

   public MultiSenseParamaterSetter(RosMainNode rosMainNode, ROS2NodeInterface ros2Node)
   {
      this.rosMainNode = rosMainNode;
      multiSenseClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(Reconfigure._TYPE);
      rosMainNode.attachServiceClient("multisense/set_parameters", multiSenseClient);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           MultisenseParameterPacket.class,
                                           ROS2Tools.IHMC_TOPIC_PREFIX + "/multisense_parameter",
                                           s -> receivedPacket(s.takeNextData()));
      publisher = ROS2Tools.createPublisher(ros2Node, MultisenseParameterPacket.class, ROS2Tools.IHMC_TOPIC_PREFIX + "/initial_multisense_parameter");
   }

   public MultiSenseParamaterSetter(RosMainNode rosMainNode2)
   {
      this.rosMainNode = rosMainNode2;
      multiSenseClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(Reconfigure._TYPE);
   }

   public boolean setupNativeROSCommunicator(double lidarSpindleVelocity)
   {
      String rosPrefix = "/opt/ros";
      if (useRosHydro(rosPrefix))
      {
         LogTools.info("Using ROS Hydro");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/hydro/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }
      else if (useRosGroovy(rosPrefix))
      {
         LogTools.info("Using ROS Groovy");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/groovy/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }
      else if (useRosFuerte(rosPrefix))
      {
         LogTools.info("Using ROS Fuerte");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/fuerte/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }
      else if (useRosIndigo(rosPrefix))
      {
         LogTools.info("Using ROS Indigo");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/indigo/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }
      else if (useRosKinetic(rosPrefix))
      {
         LogTools.info("Using ROS Kinetic");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/kinetic/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }
      else if (useRosMelodic(rosPrefix))
      {
         LogTools.info("Using ROS Melodic");
         String[] spindleSpeedShellString = {"sh", "-c", ". /opt/ros/melodic/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed "
               + lidarSpindleVelocity + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
         return shellOutSpindleSpeedCommand(spindleSpeedShellString);
      }

      throw new RuntimeException();
   }

   private boolean shellOutSpindleSpeedCommand(String[] shellCommandString)
   {
      ProcessBuilder builder = new ProcessBuilder(shellCommandString);
      try
      {
         ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
         PrintStream printStream = new PrintStream(byteArrayOutputStream);

         Process process = builder.start();
         LogTools.info("Spindle speed shellout process started");
         new ProcessStreamGobbler("ROS spindle speed shellout err", process, process.getErrorStream(), System.err).start();
         new ProcessStreamGobbler("ROS spindle speed shellout out", process, process.getInputStream(), printStream).start();
         process.waitFor();
         LogTools.info("Spindle speed shellout process finished");

         String errorString = new String(byteArrayOutputStream.toByteArray(), StandardCharsets.UTF_8);
         System.out.println(errorString);

         if (!errorString.contains("Unable to register"))
         {
            LogTools.info("ROS appears to be running and spindle is started");
            return true;
         }
         else
         {
            LogTools.info("Unable to register to ROS master. Trying again...");
            return false;
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   private boolean useRosMelodic(String rosPrefix)
   {
      return new File(rosPrefix + "/melodic").exists();
   }

   private boolean useRosFuerte(String rosPrefix)
   {
      return new File(rosPrefix + "/fuerte").exists();
   }

   private boolean useRosGroovy(String rosPrefix)
   {
      return new File(rosPrefix + "/groovy").exists();
   }

   private boolean useRosHydro(String rosPrefix)
   {
      return new File(rosPrefix + "/hydro").exists();
   }

   private boolean useRosIndigo(String rosPrefix)
   {
      return new File(rosPrefix + "/indigo").exists();
   }

   private boolean useRosKinetic(String rosPrefix)
   {
      return new File(rosPrefix + "/kinetic").exists();
   }

   public void setupMultisenseSpindleSpeedPublisher(RosMainNode rosMainNode, final double lidarSpindleVelocity)
   {
      final RosDoublePublisher rosDoublePublisher = new RosDoublePublisher(true)
      {
         @Override
         public void connected()
         {
            publish(lidarSpindleVelocity);
         }
      };
      rosMainNode.attachPublisher("/multisense/set_spindle_speed", rosDoublePublisher);
   }

   public void handleMultisenseParameters(MultisenseParameterPacket object)
   {
      if (object.getInitialize())
      {
         if (rosMainNode.isStarted())
         {
            params = rosMainNode.getParameters();
            send();

         }
      }
      else
         setMultisenseParameters(object);

   }

   private void send()
   {
      if (params == null)
      {
         //System.out.println("params are null");
         return;
      }

      publisher.publish(HumanoidMessageTools.createMultisenseParameterPacket(false,
                                                                             params.getDouble("/multisense/gain"),
                                                                             params.getDouble("/multisense/motor_speed"),
                                                                             params.getDouble("/multisense/led_duty_cycle"),
                                                                             params.getBoolean("/multisense/lighting"),
                                                                             params.getBoolean("/multisense/flash"),
                                                                             params.getBoolean("multisense/auto_exposure"),
                                                                             params.getBoolean("multisense/auto_white_balance")));
   }

   public void initializeParameterListeners()
   {

      //System.out.println("------------initialise blackfly parameteres--------------");

      rosMainNode.attachParameterListener("/multisense/motor_speed", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new motor speed received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/gain", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new gain received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/led_duty_cycle", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new dutyCycle received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/lighting", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new led received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/flash", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            // System.out.println("new flash received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/auto_exposure", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new auto expo received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/motor_speed", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new motor speed received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/auto_white_balance", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            //System.out.println("new auto white balance received");
            send();
         }
      });

   }

   public void setMultisenseResolution(RosMainNode rosMainNode)
   {
      try
      {

         Thread setupThread = new Thread()
         {
            public void run()
            {
               multiSenseClient.waitTillConnected();
               ReconfigureRequest request = multiSenseClient.getMessage();
               StrParameter resolutionParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(StrParameter._TYPE);
               resolutionParam.setName("resolution");
               //               System.out.println("Setting multisense resolution to 2048x1088");
               //               resolutionParam.setValue("2048x1088x64");
               System.out.println("Setting multisense resolution to 1024x544x128");
               resolutionParam.setValue("1024x544x256");
               request.getConfig().getStrs().add(resolutionParam);

               DoubleParameter fpsParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
               fpsParam.setName("fps");
               fpsParam.setValue(30.0);
               request.getConfig().getDoubles().add(fpsParam);

               DoubleParameter gainParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
               gainParam.setName("gain");
               gainParam.setValue(3.2);
               request.getConfig().getDoubles().add(gainParam);

               multiSenseClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
               {

                  public void onSuccess(ReconfigureResponse response)
                  {
                     System.out.println("Set resolution to " + response.getConfig().getStrs().get(0).getValue());
                  }

                  public void onFailure(RemoteException e)
                  {
                     e.printStackTrace();
                  }
               });
            }
         };

         setupThread.start();

      }
      catch (Exception e)
      {
         System.err.println(e.getMessage());
      }
   }

   public void setMultisenseParameters(MultisenseParameterPacket object)
   {

      // System.out.println("object received with gain "+ object.getGain()+" speed "+ object.getMotorSpeed()+" dutycycle"+object.getDutyCycle()+" resolution"+ object.getResolution());

      multiSenseClient.waitTillConnected();
      final ReconfigureRequest request = multiSenseClient.getMessage();
      if (object.getGain() != gain)
      {
         gain = object.getGain();
         DoubleParameter gainParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
         gainParam.setName("gain");
         gainParam.setValue(gain);
         request.getConfig().getDoubles().add(gainParam);
      }

      if (object.getMotorSpeed() != motorSpeed)
      {
         motorSpeed = object.getMotorSpeed();
         DoubleParameter motorSpeedParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
         motorSpeedParam.setName("motor_speed");
         motorSpeedParam.setValue(motorSpeed);
         request.getConfig().getDoubles().add(motorSpeedParam);
      }

      if (object.getDutyCycle() != dutyCycle)
      {
         dutyCycle = object.getDutyCycle();
         DoubleParameter dutyCycleParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
         dutyCycleParam.setName("led_duty_cycle");
         dutyCycleParam.setValue(dutyCycle);
         request.getConfig().getDoubles().add(dutyCycleParam);
      }

      if (object.getLedEnable() != ledEnable)
      {
         ledEnable = object.getLedEnable();
         BoolParameter ledEnableParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
         ledEnableParam.setName("lighting");
         ledEnableParam.setValue(ledEnable);
         request.getConfig().getBools().add(ledEnableParam);
      }

      if (object.getFlashEnable() != flashEnable)
      {
         flashEnable = object.getFlashEnable();
         BoolParameter flashEnableParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
         flashEnableParam.setName("flash");
         flashEnableParam.setValue(flashEnable);
         request.getConfig().getBools().add(flashEnableParam);
      }

      if (object.getAutoExposure() != autoExposure)
      {
         autoExposure = object.getAutoExposure();
         BoolParameter autoExposureParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
         autoExposureParam.setName("auto_exposure");
         autoExposureParam.setValue(autoExposure);
         request.getConfig().getBools().add(autoExposureParam);
      }

      if (object.getAutoWhiteBalance() != autoWhitebalance)
      {
         autoWhitebalance = object.getAutoWhiteBalance();
         BoolParameter autoWhiteBalanceParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
         autoWhiteBalanceParam.setName("auto_white_balance");
         autoWhiteBalanceParam.setValue(autoWhitebalance);
         request.getConfig().getBools().add(autoWhiteBalanceParam);
      }

      new Thread()
      {
         @Override
         public void run()
         {
            multiSenseClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
            {

               public void onSuccess(ReconfigureResponse response)
               {
                  System.out.println("successful" + response.getConfig().getDoubles().get(0).getValue());
               }

               public void onFailure(RemoteException e)
               {
                  e.printStackTrace();
               }
            });
         }

      }.start();
   }

   public void receivedPacket(MultisenseParameterPacket object)
   {
      handleMultisenseParameters(object);
   }

}
