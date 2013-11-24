package us.ihmc.darpaRoboticsChallenge.gazebo;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.ros.exception.RemoteException;
import org.ros.internal.message.RawMessage;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseListener;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotDampingParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.RosStringPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.ros.ROSSandiaJointMap;
import us.ihmc.robotSide.RobotSide;
import atlas_msgs.SetJointDamping;
import atlas_msgs.SetJointDampingRequest;
import atlas_msgs.SetJointDampingResponse;
import cern.colt.Arrays;
import dynamic_reconfigure.Config;
import dynamic_reconfigure.DoubleParameter;

public class RosAtlasSettingsSetter
{
   private final RosMainNode rosMainNode;
   private final RosStringPublisher modePublisher = new RosStringPublisher(true, "nominal");
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> atlasDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> sandiaHandDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);
   
   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
   
   private final RosServiceClient<Config, Config> fishEyeClient = new RosServiceClient<Config, Config>(Config._TYPE);
   
   public RosAtlasSettingsSetter(String rosMasterURI)
   {
      try
      {
         rosMainNode = new RosMainNode(new URI(rosMasterURI), "RosAtlasSettingsSetter");
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException(e);
      }

      rosMainNode.attachPublisher("/atlas/mode", modePublisher);
      //rosMainNode.attachServiceClient("/atlas/set_joint_damping", atlasDampingClient);
     // rosMainNode.attachServiceClient("/sandia_hands/set_joint_damping", sandiaHandDampingClient);
      rosMainNode.attachServiceClient("blackfly/set_parameters", fishEyeClient);
      rosMainNode.execute();

   }
   
   public void setFishEyeParameters()
   {
      
      fishEyeClient.waitTillConnected();
      System.out.println("got here");
      Config request = fishEyeClient.getMessage();
      DoubleParameter frameRateDoubleParam = nodeConfig.getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      frameRateDoubleParam.setName("prop_frame_rate");
      frameRateDoubleParam.setValue(30.0);
      request.getDoubles().add(frameRateDoubleParam);
       
      fishEyeClient.call(request, new ServiceResponseListener<Config>()
      {

         public void onSuccess(Config response)
         {
            System.out.println("success" + response.getDoubles().get(0).getValue());
         }

         public void onFailure(RemoteException e)
         {
            throw new RuntimeException(e);
         }
      });
   }

   public void setPositionControlDampingParameters()
   {
      atlasDampingClient.waitTillConnected();

      SetJointDampingRequest request = atlasDampingClient.getMessage();

      final double[] dampingParameters = new double[ROSAtlasJointMap.numberOfJoints];
      for (int i = 0; i < dampingParameters.length; i++)
      {
         dampingParameters[i] = DRCRobotDampingParameters.getAtlasDampingForPositionControl(i);
      }

      request.setDampingCoefficients(dampingParameters);

      atlasDampingClient.call(request, new ServiceResponseListener<SetJointDampingResponse>()
      {

         public void onSuccess(SetJointDampingResponse response)
         {
            System.out.println("Set joint damping for Atlas to " + Arrays.toString(dampingParameters) + " "  + response.getStatusMessage());
         }

         public void onFailure(RemoteException e)
         {
            throw new RuntimeException(e);
         }
      });
   }

   public void setAtlasDampingParameters()
   {
      atlasDampingClient.waitTillConnected();

      SetJointDampingRequest request = atlasDampingClient.getMessage();

      final double[] dampingParameters = new double[ROSAtlasJointMap.numberOfJoints];
      for (int i = 0; i < dampingParameters.length; i++)
      {
         dampingParameters[i] = DRCRobotDampingParameters.getAtlasDamping(i);
      }

      request.setDampingCoefficients(dampingParameters);

      atlasDampingClient.call(request, new ServiceResponseListener<SetJointDampingResponse>()
      {

         public void onSuccess(SetJointDampingResponse response)
         {
            System.out.println("Set joint damping for Atlas to " + Arrays.toString(dampingParameters) + " "  + response.getStatusMessage());
         }

         public void onFailure(RemoteException e)
         {
            throw new RuntimeException(e);
         }
      });
   }
   
   public void setSandiaHandDampingParameters()
   {
      sandiaHandDampingClient.waitTillConnected();

      SetJointDampingRequest request = sandiaHandDampingClient.getMessage();

      final double[] dampingParameters = new double[ROSAtlasJointMap.numberOfJoints]; // Message reuse on OSRF side, it is 28 (number of atlas joints) long
      for (int i = 0; i < ROSSandiaJointMap.numberOfJointsPerHand; i++)
      {
         dampingParameters[i] = DRCRobotDampingParameters.getSandiaHandDamping(RobotSide.LEFT, i);
      }
      
      for (int i = 0; i < ROSSandiaJointMap.numberOfJointsPerHand; i++)
      {
         dampingParameters[i + ROSSandiaJointMap.numberOfJointsPerHand] = DRCRobotDampingParameters.getSandiaHandDamping(RobotSide.RIGHT, i);
      }

      request.setDampingCoefficients(dampingParameters);

      sandiaHandDampingClient.call(request, new ServiceResponseListener<SetJointDampingResponse>()
      {

         public void onSuccess(SetJointDampingResponse response)
         {
            System.out.println("Set joint damping for the Sandia Hands to " + Arrays.toString(dampingParameters));
         }

         public void onFailure(RemoteException e)
         {
            throw new RuntimeException(e);
         }
      });
   }

   public static void main(String[] args)
   {
      RosAtlasSettingsSetter rosAtlasSettingsSetter = new RosAtlasSettingsSetter(DRCConfigParameters.ROS_MASTER_URI);
      //rosAtlasSettingsSetter.setAtlasDampingParameters();
      rosAtlasSettingsSetter.setFishEyeParameters();
      System.exit(0);
   }
}
