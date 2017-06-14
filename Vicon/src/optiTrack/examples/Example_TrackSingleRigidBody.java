package optiTrack.examples;

import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.time.CallFrequencyCalculator;

public class Example_TrackSingleRigidBody implements MocapRigidbodiesListener
{
   private int rigidBodyIdToTrack = 1;

   private YoVariableRegistry registry = new YoVariableRegistry("MOCAP");
   private CallFrequencyCalculator frequencyCalculator = new CallFrequencyCalculator(registry, "");

   public Example_TrackSingleRigidBody()
   {
      MocapDataClient udpMulticastClient = new MocapDataClient();
      udpMulticastClient.registerRigidBodiesListener(this);
   }

   public static void main(String args[])
   {
      new Example_TrackSingleRigidBody();
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (MocapRigidBody rb : listOfRigidbodies)
      {
         if (rb.getId() == rigidBodyIdToTrack)
         {
        	 System.out.println(rb.toString());
         }
      }
   }
}
