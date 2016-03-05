package optiTrack.examples;

import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;



public class Example_MocapClientRigidBodiesListener implements MocapRigidbodiesListener
{
   int rigidBodyIdToTrack = 1;

   public Example_MocapClientRigidBodiesListener()
   {
      // UDPMilticastCommandClient udpMilticastCommandClient = new UDPMilticastCommandClient();
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
   }

   public int getTrackingId()
   {
      return rigidBodyIdToTrack;
   }

   public static void main(String args[])
   {
      Example_MocapClientRigidBodiesListener mocapClientExample = new Example_MocapClientRigidBodiesListener();
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (MocapRigidBody rb : listOfRigidbodies)
      {
		  System.out.println(rb.toString());
      }
   }
}


//~ Formatted by Jindent --- http://www.jindent.com
