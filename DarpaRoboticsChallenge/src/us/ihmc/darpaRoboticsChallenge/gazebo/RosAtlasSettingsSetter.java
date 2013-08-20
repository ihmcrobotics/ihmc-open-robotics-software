package us.ihmc.darpaRoboticsChallenge.gazebo;

import java.net.URI;
import java.net.URISyntaxException;

import org.ros.exception.RemoteException;
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
      rosAtlasSettingsSetter.setAtlasDampingParameters();
      System.exit(0);
   }
}
