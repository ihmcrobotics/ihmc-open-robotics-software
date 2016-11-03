package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CoactiveBehaviorsNetworkManager;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SearchForValveBehavior extends AbstractBehavior implements CoactiveDataListenerInterface
{
   private Point3d valveCenter;

   private double valveRotation;
   private double valveRadius;
   private boolean recievedNewValveLocation = false;
   private CoactiveBehaviorsNetworkManager networkManager;

   public SearchForValveBehavior(BehaviorCommunicationBridge behaviorCommunicationBridge)
   {
      super("SearchForSpehereFar", behaviorCommunicationBridge);
      networkManager = new CoactiveBehaviorsNetworkManager(behaviorCommunicationBridge, behaviorCommunicationBridge);
      networkManager.addListeners(this);
   }

   @Override
   public void doControl()
   {
      //nothing in do control. simply waiting for the valve location to be sent.
   }

   @Override
   public boolean isDone()
   {
      return recievedNewValveLocation;
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      recievedNewValveLocation = false;
   }

   public Point3d getValveCenter()
   {
      return valveCenter;
   }

   public double getValveRotation()
   {
      return valveRotation;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {

      if (data.key.equalsIgnoreCase("ValveLocation"))
      {
         valveCenter = new Point3d();
         valveCenter.x = ((double[]) data.dataObject)[0];
         valveCenter.y = ((double[]) data.dataObject)[1];
         valveCenter.z = ((double[]) data.dataObject)[2];
         valveRotation = ((double[]) data.dataObject)[3];
         valveRadius = ((double[]) data.dataObject)[4];
         recievedNewValveLocation = true;
      }

   }

}
