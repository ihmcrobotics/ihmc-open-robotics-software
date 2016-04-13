package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;

import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class KickBallBehaviorCoactiveElement implements CoactiveElement
{
   private final YoVariableRegistry userInterfaceWritableRegistry = new YoVariableRegistry("UserInterfaceSide");
   private final YoVariableRegistry machineWritableRegistry = new YoVariableRegistry("MachineSide");

   private final IntegerYoVariable userInterfaceSideCount = new IntegerYoVariable("userInterfaceSideCount", userInterfaceWritableRegistry);
   private final BooleanYoVariable abortClicked = new BooleanYoVariable("abortClicked", userInterfaceWritableRegistry);

   private final IntegerYoVariable machineSideCount = new IntegerYoVariable("machineSideCount", machineWritableRegistry);
   private final IntegerYoVariable abortCount = new IntegerYoVariable("abortCount", machineWritableRegistry);
   private final BooleanYoVariable abortAcknowledged = new BooleanYoVariable("abortAcknowledged", machineWritableRegistry);

   private KickBallBehavior kickBallBehavior;

   private JFrame jFrame;
   private JButton jButton;

   public void setKickBallBehavior(KickBallBehavior kickBallBehavior)
   {
      this.kickBallBehavior = kickBallBehavior;
   }

   @Override
   public void initializeUserInterfaceSide()
   {
      if (jFrame == null)
      {
         jFrame = new JFrame();
         jFrame.setSize(600, 600);

         jButton = new JButton("Abort");
         jFrame.getContentPane().add(jButton);

         jButton.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent arg0)
            {
               abortClicked.set(true);
            }
         });

         jFrame.setVisible(true);
      }
      userInterfaceSideCount.set(10);
   }

   @Override
   public void updateUserInterfaceSide()
   {
      userInterfaceSideCount.increment();

      if ((abortClicked.getBooleanValue()) && (abortAcknowledged.getBooleanValue()))
      {
         abortClicked.set(false);
      }
   }

   @Override
   public YoVariableRegistry getUserInterfaceWritableYoVariableRegistry()
   {
      return userInterfaceWritableRegistry;
   }

   @Override
   public void initializeMachineSide()
   {
      machineSideCount.set(100);
   }

   @Override
   public void updateMachineSide()
   {
      if (abortAcknowledged.getBooleanValue() && (!abortClicked.getBooleanValue()))
      {
         abortAcknowledged.set(false);
      }

      if ((abortClicked.getBooleanValue()) && (!abortAcknowledged.getBooleanValue()))
      {
         if (kickBallBehavior != null)
         {
            kickBallBehavior.abort();
         }
         abortCount.increment();
         abortAcknowledged.set(true);
      }

      machineSideCount.increment();
   }

   @Override
   public YoVariableRegistry getMachineWritableYoVariableRegistry()
   {
      return machineWritableRegistry;
   }
}
