package optiTrack.Scs;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Example_Mocap_SCS implements MocapRigidbodiesListener
{
   private MocapDataClient mocapDataClient;
   YoVariableRegistry registry = new YoVariableRegistry("mainRegistry");
   SimulationConstructionSet scs;

   public Example_Mocap_SCS()
   {
      setupUI();
   }

   public void setupUI()
   {
      JFrame frame = new JFrame("FrameDemo");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      JButton startSCS = new JButton("Start SCS");
      startSCS.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            Robot robot = new Robot("Mocap_Robot");
            robot.addYoVariableRegistry(registry);

            mocapDataClient = new MocapDataClient();
            mocapDataClient.registerRigidBodiesListener(Example_Mocap_SCS.this);

            scs = new SimulationConstructionSet(robot);
            scs.setMaxBufferSize(64000);
//            scs.setDT(0.0001, 75);
            Thread myThread = new Thread(scs);
            myThread.start();
         }
      });

      frame.getContentPane().add(startSCS, BorderLayout.CENTER);
      frame.pack();
      frame.setVisible(true);
   }

   public static void main(String args[])
   {
      new Example_Mocap_SCS();
   }

   ArrayList<ScsMocapRigidBody> listOfRbs = new ArrayList<>();

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (MocapRigidBody rb : listOfRigidbodies)
      {
         boolean exists = false;
         for (ScsMocapRigidBody scsRb : listOfRbs)
         {
            if (rb.getId() == scsRb.getId())
            {
               scsRb.update(rb);
               exists = true;
               break;
            }
         }

         if (!exists)
         {
            System.out.println("Rigidbody " + rb.getId() + " does not exist locally. Creating one nad adding it to the local registry");
            ScsMocapRigidBody scsMocapRigidBody = new ScsMocapRigidBody(rb);
            listOfRbs.add(scsMocapRigidBody);
            registry.addChild(scsMocapRigidBody.getRegistry());
         }
      }
      
      if(scs != null)
      {
         scs.tickAndUpdate();
      }
      
   }
}
