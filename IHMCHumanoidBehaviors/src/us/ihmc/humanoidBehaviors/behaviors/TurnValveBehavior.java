package us.ihmc.humanoidBehaviors.behaviors;

//~--- non-JDK imports --------------------------------------------------------

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
//~--- JDK imports ------------------------------------------------------------
import java.io.InputStream;

public class TurnValveBehavior extends BehaviorInterface
{
   private final double MAX_WRIST_FORCE_THRESHOLD_N = 40.0;
   private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.13, 0.0, 0.64);
   private Vector3d valveLocation = new Vector3d();
   private Vector3d valveOffsetInWorldFrame = new Vector3d();
   private final ArrayList<BehaviorInterface> behaviorQueue = new ArrayList<>();
   private final ScriptBehavior scriptBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private Point3d targetWalkLocation;
   private final YoFrameOrientation targetWalkOrientation;
   private InputStream scriptResourceStream;
   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
   private RigidBodyTransform worldToValveTransform;
   private YoFrameOrientation valveOrientation;
   private BehaviorInterface currentBehavior;
   private boolean isDone;
   
   private ForceSensorDataHolder globalForceSensorData;
   private Wrench rightWristSensorWrench;
   private Vector3d rightWristSensorForce;
   private final DoubleYoVariable yoTime;

   // private final ModifiableValveModel valveModel;
   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, ForceSensorDataHolder forceSensorDataHolder)
   {
      super(outgoingCommunicationBridge);
      targetWalkLocation = new Point3d();
      valveOrientation = new YoFrameOrientation(behaviorName + "ValveOrientation", ReferenceFrame.getWorldFrame(), registry);
      targetWalkOrientation = new YoFrameOrientation(behaviorName + "WalkToOrientation", ReferenceFrame.getWorldFrame(), registry);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames);
      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
      globalForceSensorData = forceSensorDataHolder;
      getForceSensorDefinition(fullRobotModel);
      rightWristSensorWrench = new Wrench();
      rightWristSensorForce = new Vector3d();
      this.yoTime = yoTime;
      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   
   public void getForceSensorDefinition(FullRobotModel fullRobotModel){
      for(ForceSensorDefinition currentForceSensorDefinition : fullRobotModel.getForceSensorDefinitions()){
         System.out.println("TurnValveBehavior: " + currentForceSensorDefinition.toString());
      }
   };
   
   public void updateWristSensorValues()
   {
      String rightWristForceSensorName = "r_arm_wrx";
            //AtlasSensorInformation.handForceSensorNames.get(RobotSide.LEFT);
      ForceSensorData rightWristForceSensor = globalForceSensorData.getByName(rightWristForceSensorName);
      rightWristForceSensor.packWrench(rightWristSensorWrench);
      
      rightWristSensorWrench.packLinearPart(rightWristSensorForce);
   }
   
   @Override
   public void doControl()
   {
      updateWristSensorValues();
      
      double rightWristForce = rightWristSensorForce.length();
      
      if ( rightWristForce > MAX_WRIST_FORCE_THRESHOLD_N )
      {
         System.out.println("TurnValveBehavior: MAX WRIST FORCE EXCEEDED!  Force Magnitude =  " + rightWristForce);
      }
      

      if(this.yoTime.getDoubleValue() % 7.0 == 0.0)
      {
         System.out.println( "TurnValveBehavior: right wrist Fx : " + rightWristSensorWrench.getLinearPartX());
         System.out.println( "TurnValveBehavior: right wrist Fy : " + rightWristSensorWrench.getLinearPartY());
         System.out.println( "TurnValveBehavior: right wrist Fz : " + rightWristSensorWrench.getLinearPartZ());
      }
      
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable())
      {
         ScriptBehaviorInputPacket scriptBehaviorInputPacket = scriptBehaviorInputPacketListener.getNewestPacket();
         System.out.println("TurnValveBehavior: New Script Behavior Input Packet Received: " + scriptBehaviorInputPacket);

         worldToValveTransform = scriptBehaviorInputPacket.getReferenceTransform();

         setValveLocationAndOrientation(worldToValveTransform);
         setTargetWalkLocationAndOrientation(valveLocation, valveOrientation);

         String scriptName = scriptBehaviorInputPacket.getScriptName();

         scriptResourceStream = this.getClass().getClassLoader().getResourceAsStream(scriptName);

         if (scriptResourceStream == null)
            System.out.println("Script Resource Stream is null. Can't load script!");
         else
            System.out.println("Script " + scriptBehaviorInputPacket.getScriptName() + " loaded.");

         scriptBehavior.setScriptInputs(scriptResourceStream, worldToValveTransform);
      }

      if (currentBehavior.isDone())
      {
         System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is done.");
         currentBehavior.finalize();
         if (!behaviorQueue.isEmpty())
         {
            currentBehavior = behaviorQueue.remove(0);
            currentBehavior.initialize();
            System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is starting.");
         }
         else
         {
            isDone = true;
         }
      }
      currentBehavior.doControl();
   }

   private void setValveLocationAndOrientation(RigidBodyTransform worldToValveTransform)
   {
      worldToValveTransform.get(valveLocation);
      valveOrientation.set(worldToValveTransform);
   }

   private void setTargetWalkLocationAndOrientation(Vector3d valveLocation, YoFrameOrientation valveOrientation)
   {
      targetWalkLocation.set(valveLocation);
      worldToValveTransform.transform(valveInteractionOffsetInValveFrame, valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveOffsetInWorldFrame);
      targetWalkLocation.add(valveOffsetInWorldFrame);
      targetWalkOrientation.setYawPitchRoll(valveOrientation.getYaw().getDoubleValue() + 0.5 * Math.PI, valveOrientation.getPitch().getDoubleValue(),
            valveOrientation.getRoll().getDoubleValue());
      walkToLocationBehavior.setTarget(targetWalkLocation, targetWalkOrientation);
      System.out.println("TurnValveBehavior: Turn Valve Location Updated:" + valveLocation);
      System.out.println("TurnValveBehavior: Target Walk to Location Updated:" + targetWalkLocation);
   }

   private boolean inputsSupplied()
   {
      System.out.println("inputsSupplied: " + ((scriptResourceStream != null) && (worldToValveTransform != null)));
      return ((scriptResourceStream != null) && (worldToValveTransform != null));
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      currentBehavior.passReceivedNetworkProcessorObjectToChildBehaviors(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
   }

   @Override
   public void stop()
   {
      currentBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      // currentBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      currentBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      currentBehavior.resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return isDone;
   }

   @Override
   public void finalize()
   {
      currentBehavior.finalize();
   }

   @Override
   public void initialize()
   {
      if (scriptBehavior != null)
      {
         scriptBehavior.initialize();
      }
      if (walkToLocationBehavior != null)
      {
         walkToLocationBehavior.initialize();
      }

      behaviorQueue.clear();
      behaviorQueue.add(walkToLocationBehavior);
      behaviorQueue.add(scriptBehavior);
      currentBehavior = behaviorQueue.remove(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return currentBehavior.hasInputBeenSet();
   }
}
