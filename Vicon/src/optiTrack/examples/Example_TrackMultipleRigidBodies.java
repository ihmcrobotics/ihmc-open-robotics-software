package optiTrack.examples;

import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.time.CallFrequencyCalculator;

public class Example_TrackMultipleRigidBodies implements MocapRigidbodiesListener
{
   private YoVariableRegistry registry = new YoVariableRegistry("MOCAP");
   private CallFrequencyCalculator frequencyCalculator = new CallFrequencyCalculator(registry, "");

   public Example_TrackMultipleRigidBodies()
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      System.out.println("\n\n>> START DATA RECEIVED: ");
      System.out.println("# of RigidBodies: " + listOfRigidbodies.size());

      for (MocapRigidBody rigidBody : listOfRigidbodies)
      {
         System.out.println(rigidBody.toString());
      }

      System.out.println("Update rate: " + frequencyCalculator.determineCallFrequency() + " Hz");
      System.out.println("<< END DATA RECEIVED ");
   }

   public static void main(String args[])
   {
      new Example_TrackMultipleRigidBodies();
   }
}
