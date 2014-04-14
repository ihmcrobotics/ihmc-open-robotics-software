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

import us.ihmc.atlas.AtlasDampingParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.RosStringPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.AtlasOrderedJointMap;
import us.ihmc.darpaRoboticsChallenge.ros.ROSSandiaJointMap;
import us.ihmc.robotSide.RobotSide;
import atlas_msgs.SetJointDamping;
import atlas_msgs.SetJointDampingRequest;
import atlas_msgs.SetJointDampingResponse;
import cern.colt.Arrays;
import dynamic_reconfigure.DoubleParameter;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;

public class RosAtlasSettingsSetter
{
   private final RosMainNode rosMainNode;
   private final RosStringPublisher modePublisher = new RosStringPublisher(true, "nominal");
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> atlasDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> sandiaHandDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);
   
   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
   
   private final RosServiceClient<ReconfigureRequest, ReconfigureResponse> fishEyeClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(Reconfigure._TYPE);
   
   public RosAtlasSettingsSetter(String rosMasterURI, final RosMainNode rosMainNode)
   {
      this.rosMainNode = rosMainNode;
      try{
         rosMainNode.attachServiceClient("blackfly/set_parameters", fishEyeClient);
      } catch (Exception e)
      {
         System.err.println("Could Not connect to FishEye Node");
      }
    

   }
   
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

    //rosMainNode.attachPublisher("/atlas/mode", modePublisher);
    //rosMainNode.attachServiceClient("/atlas/set_joint_damping", atlasDampingClient);
   // rosMainNode.attachServiceClient("/sandia_hands/set_joint_damping", sandiaHandDampingClient);
    try{
       rosMainNode.attachServiceClient("blackfly/set_parameters", fishEyeClient);
    } catch (Exception e)
    {
       System.err.println("Could Not connect to FishEye Node");
    }
    rosMainNode.execute();
   }

   public void setFishEyeFrameRate(double frameRate)
   {
      System.out.println("got here");
      fishEyeClient.waitTillConnected();
      System.out.println("got here");
      ReconfigureRequest request = fishEyeClient.getMessage();
      DoubleParameter frameRateDoubleParam = nodeConfig.getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      frameRateDoubleParam.setName("prop_frame_rate");
      frameRateDoubleParam.setValue(frameRate);
      request.getConfig().getDoubles().add(frameRateDoubleParam);
           
      fishEyeClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
      {

         public void onSuccess(ReconfigureResponse response)
         {
            System.out.println("success" + response.getConfig().getDoubles().get(1).getValue());
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

      final double[] dampingParameters = new double[AtlasOrderedJointMap.numberOfJoints];
      for (int i = 0; i < dampingParameters.length; i++)
      {
         dampingParameters[i] = AtlasDampingParameters.getAtlasDampingForPositionControl(i);
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

      final double[] dampingParameters = new double[AtlasOrderedJointMap.numberOfJoints];
      for (int i = 0; i < dampingParameters.length; i++)
      {
         dampingParameters[i] = AtlasDampingParameters.getAtlasDamping(i);
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

      final double[] dampingParameters = new double[AtlasOrderedJointMap.numberOfJoints]; // Message reuse on OSRF side, it is 28 (number of atlas joints) long
      for (int i = 0; i < ROSSandiaJointMap.numberOfJointsPerHand; i++)
      {
         dampingParameters[i] = AtlasDampingParameters.getSandiaHandDamping(RobotSide.LEFT, i);
      }
      
      for (int i = 0; i < ROSSandiaJointMap.numberOfJointsPerHand; i++)
      {
         dampingParameters[i + ROSSandiaJointMap.numberOfJointsPerHand] = AtlasDampingParameters.getSandiaHandDamping(RobotSide.RIGHT, i);
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
      rosAtlasSettingsSetter.setFishEyeFrameRate(10.0);
      System.exit(0);
   }
}
