package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import java.io.File;
import java.io.IOException;

import org.ros.exception.RemoteException;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseListener;

import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.MultisenseParameterPacket;
import us.ihmc.utilities.processManagement.ProcessStreamGobbler;
import us.ihmc.utilities.ros.RosDoublePublisher;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import dynamic_reconfigure.BoolParameter;
import dynamic_reconfigure.DoubleParameter;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;
import dynamic_reconfigure.StrParameter;

public class MultiSenseParamaterSetter
{
   private static double gain;
   private static double motorSpeed;
   private static boolean ledEnable;
   private static boolean flashEnable;
   
   private static final RosServiceClient<ReconfigureRequest, ReconfigureResponse> multiSenseClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(
         Reconfigure._TYPE);
   
   public static void setupNativeROSCommunicator(RosNativeNetworkProcessor rosNativeNetworkProcessor, double lidarSpindleVelocity)
   {
      if (DRCLocalConfigParameters.IS_HEAD_ATTACHED)
      {
         String rosPrefix = "/opt/ros";
         if (useRosHydro(rosPrefix))
         {
            System.out.println("using hydro");
            String[] hydroSpindleSpeedShellString = {"sh", "-c",
                  ". /opt/ros/hydro/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed " + lidarSpindleVelocity
                        + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
            shellOutSpindleSpeedCommand(hydroSpindleSpeedShellString);
         }
         else if (useRosGroovy(rosPrefix))
         {
            System.out.println("using groovy");
            String[] groovySpindleSpeedShellString = {"sh", "-c",
                  ". /opt/ros/groovy/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed " + lidarSpindleVelocity
                  + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
            shellOutSpindleSpeedCommand(groovySpindleSpeedShellString);
         }
         else if (useRosFuerte(rosPrefix))
         {
            System.out.println("using fuerte");
            String[] fuerteSpindleSpeedShellString = {"sh", "-c",
                  ". /opt/ros/fuerte/setup.sh; rosrun dynamic_reconfigure dynparam set /multisense motor_speed " + lidarSpindleVelocity
                  + "; rosrun dynamic_reconfigure dynparam set /multisense network_time_sync true"};
            shellOutSpindleSpeedCommand(fuerteSpindleSpeedShellString);
         }
      }
   }
   
   private static void shellOutSpindleSpeedCommand(String[] shellCommandString)
   {
      ProcessBuilder builder = new ProcessBuilder(shellCommandString);
      try
      {
         Process p = builder.start();
         System.out.println("Process started.");
         new ProcessStreamGobbler("ROS Shellout", p.getErrorStream(), System.err).start();
         new ProcessStreamGobbler("ROS Shellout", p.getInputStream(), System.out).start();
         p.waitFor();
         System.out.println("Process done.");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private static boolean useRosFuerte(String rosPrefix)
   {
      return new File(rosPrefix + "/fuerte").exists();
   }

   private static boolean useRosGroovy(String rosPrefix)
   {
      return new File(rosPrefix + "/groovy").exists();
   }

   private static boolean useRosHydro(String rosPrefix)
   {
      return new File(rosPrefix + "/hydro").exists();
   }

   public static void setupMultisenseSpindleSpeedPublisher(RosMainNode rosMainNode, final double lidarSpindleVelocity)
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
   
   public static void initialize(RosMainNode rosMainNode){
      
      rosMainNode.attachServiceClient("multisense/set_parameters", multiSenseClient);
   }
   
   public static void setMultisenseResolution(RosMainNode rosMainNode)
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
               //resolutionParam.setValue("2048x1088x64");
               //System.out.println("Setting multisense resolution to 2048x1088");
               System.out.println("Setting multisense resolution to 1024x544x64");
               resolutionParam.setValue("1024x544x64");
               request.getConfig().getStrs().add(resolutionParam);

               DoubleParameter fpsParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
               fpsParam.setName("fps");
               fpsParam.setValue(30.0);
               request.getConfig().getDoubles().add(fpsParam);
               
               
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
   
   

   public static void setMultisenseParameters(MultisenseParameterPacket object)
   {
      gain = object.getGain();
      motorSpeed = object.getMotorSpeed();
      ledEnable = object.isLedEnable();
      flashEnable = object.isFlashEnable();
      System.out.println("object received with gain "+ gain+" speed "+ motorSpeed+"led "+ ledEnable +"flash"+flashEnable);
      multiSenseClient.waitTillConnected();
      ReconfigureRequest request = multiSenseClient.getMessage();
      DoubleParameter gainParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      gainParam.setName("gain");
      gainParam.setValue(gain);
      request.getConfig().getDoubles().add(gainParam);
      
      DoubleParameter motorSpeedParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      motorSpeedParam.setName("motor_speed");
      motorSpeedParam.setValue(motorSpeed);
      request.getConfig().getDoubles().add(motorSpeedParam);
      
      BoolParameter ledEnableParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
      ledEnableParam.setName("lighting");
      ledEnableParam.setValue(ledEnable);
      request.getConfig().getBools().add(ledEnableParam);
      
      BoolParameter flashEnableParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(BoolParameter._TYPE);
      flashEnableParam.setName("flash");
      flashEnableParam.setValue(flashEnable);
      request.getConfig().getBools().add(flashEnableParam);
      
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
}
