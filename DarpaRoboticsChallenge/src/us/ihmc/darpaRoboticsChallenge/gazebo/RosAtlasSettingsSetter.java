package us.ihmc.darpaRoboticsChallenge.gazebo;

import java.net.URI;
import java.net.URISyntaxException;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import cern.colt.Arrays;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosMainNode;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosServiceClient;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosStringPublisher;
import atlas_msgs.SetJointDamping;
import atlas_msgs.SetJointDampingRequest;
import atlas_msgs.SetJointDampingResponse;

public class RosAtlasSettingsSetter
{
   private final RosMainNode rosMainNode;
   private final RosStringPublisher modePublisher = new RosStringPublisher(true, "nominal");
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> atlasDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);
   private final RosServiceClient<atlas_msgs.SetJointDampingRequest, atlas_msgs.SetJointDampingResponse> sandiaHandDampingClient = new RosServiceClient<SetJointDampingRequest, SetJointDampingResponse>(
         SetJointDamping._TYPE);

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
      rosMainNode.attachServiceClient("/atlas/set_joint_damping", atlasDampingClient);
      rosMainNode.attachServiceClient("/sandia_hands/set_joint_damping", sandiaHandDampingClient);
      rosMainNode.execute();

   }


   public void setAtlasDampingParameters()
   {
      atlasDampingClient.waitTillConnected();

      SetJointDampingRequest request = atlasDampingClient.getMessage();

      final double[] dampingParameters = new double[28];
      for (int i = 0; i < dampingParameters.length; i++)
      {
         dampingParameters[i] = 0.1;
      }

      request.setDampingCoefficients(dampingParameters);

      atlasDampingClient.call(request, new ServiceResponseListener<SetJointDampingResponse>()
      {

         public void onSuccess(SetJointDampingResponse response)
         {
            System.out.println("Set joint damping for Atlas to " + Arrays.toString(dampingParameters));
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

      final double[] dampingParameters = new double[28];
      for (int i = 0; i < 24; i++)
      {
         dampingParameters[i] = 1.0;
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
      rosAtlasSettingsSetter.setAtlasDampingParameters();
      System.exit(0);
   }
}
