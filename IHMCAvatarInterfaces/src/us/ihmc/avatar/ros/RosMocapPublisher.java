package us.ihmc.avatar.ros;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.time.CallFrequencyCalculator;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;

public class RosMocapPublisher implements MocapRigidbodiesListener, Runnable
{
      private YoVariableRegistry registry = new YoVariableRegistry("MOCAP");
      private CallFrequencyCalculator frequencyCalculator = new CallFrequencyCalculator(registry, "");
      
      RosMainNode mainNode;
      RosTf2Publisher tfPublisher;



      public RosMocapPublisher()
      {         

         try
         {
            mainNode = new RosMainNode(new URI("http://10.7.4.102:11311"), getClass().getSimpleName());
            tfPublisher = new RosTf2Publisher(false);
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
         MocapDataClient mocapDataClient = new MocapDataClient();
         mocapDataClient.registerRigidBodiesListener(this);
      }

      @Override
      public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
      {
         if(!mainNode.isStarted())
            return;
         for (MocapRigidBody rigidBody : listOfRigidbodies)
         {
            RigidBodyTransform tmpTransform = new RigidBodyTransform();
            tmpTransform.setTranslation(rigidBody.xPosition,rigidBody.yPosition,rigidBody.zPosition);
            tmpTransform.setRotation(new Quaternion(rigidBody.qx, rigidBody.qy, rigidBody.qz, rigidBody.qw));           
            tfPublisher.publish(tmpTransform, mainNode.getCurrentTime().totalNsecs(), "/mocap_world", "mocap/rigidBody"+rigidBody.getId());
         }

         //System.out.println("Update rate: " + frequencyCalculator.determineCallFrequency() + " Hz");
      }
      
      public void run()
      {
         mainNode.attachPublisher("/tf", tfPublisher);
         mainNode.execute();
      }


      public static void main(String[] arg) throws URISyntaxException
      {
         RosMocapPublisher publisher = new RosMocapPublisher();
         new Thread(publisher).start();
      }
}

   
   
   
   
   
