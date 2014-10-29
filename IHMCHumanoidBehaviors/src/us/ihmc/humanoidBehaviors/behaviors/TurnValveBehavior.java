package us.ihmc.humanoidBehaviors.behaviors;

//~--- non-JDK imports --------------------------------------------------------

//~--- JDK imports ------------------------------------------------------------
import java.io.InputStream;
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
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.FirstOrderBandPassFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class TurnValveBehavior extends BehaviorInterface
{
   private final double forceSensorMinPassThroughFreq_Hz = 0.1;
   private final double forceSensorMaxPassThroughFreq_Hz = 50.0;
   
   private final double MAX_WRIST_FORCE_THRESHOLD_N = 40.0; // Sensor reads ~ 100 N after stand-prep with Robotiq hands
   private final double MAX_CAPTURE_POINT_ERROR = 0.02; // Reasonable value < 0.01   Max < 0.02

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

   private final String rightWristForceSensorName = "r_arm_wrx"; //AtlasSensorInformation.handForceSensorNames.get(RobotSide.RIGHT);
   private ForceSensorDataHolder globalForceSensorData;
   private Wrench rightWristWrench;
   private Vector3d rightWristForce;
   private final DoubleYoVariable rightWristForceMagnitude;
   private final FirstOrderBandPassFilteredYoVariable rightWristForceBandPassFiltered;


   private Double maxObservedWristForce = 0.0;
   private final DoubleYoVariable yoTime;
   private final double DT;

   private final YoFramePoint2d yoCapturePoint;
   private final YoFramePoint2d yoDesiredCapturePoint;
   private final DoubleYoVariable capturePointErrorMag;
   private Double maxObservedCapturePointError = 0.0;

   // private final ModifiableValveModel valveModel;
   
   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, YoFramePoint2d yoCapturePoint, YoFramePoint2d yoDesiredCapturePoint, ForceSensorDataHolder forceSensorDataHolder, double DT)
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
      rightWristWrench = new Wrench();
      rightWristForce = new Vector3d();
      rightWristForceMagnitude = new DoubleYoVariable("rightWristForceMag", registry);
      
      rightWristForceBandPassFiltered = new FirstOrderBandPassFilteredYoVariable("rightWristForceMag", "", forceSensorMinPassThroughFreq_Hz, forceSensorMaxPassThroughFreq_Hz, yoTime, registry);
      
      this.yoTime = yoTime;
      this.DT = DT;

      this.yoCapturePoint = yoCapturePoint;
      this.yoDesiredCapturePoint = yoDesiredCapturePoint;
      this.capturePointErrorMag = new DoubleYoVariable("capturePointError", registry);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   public void getForceSensorDefinition(FullRobotModel fullRobotModel)
   {
      for (ForceSensorDefinition currentForceSensorDefinition : fullRobotModel.getForceSensorDefinitions())
      {
         System.out.println("TurnValveBehavior: " + currentForceSensorDefinition.toString());
      }
   };

   public void updateWristSensorValues()
   {
      ForceSensorData rightWristForceSensorData = globalForceSensorData.getByName(rightWristForceSensorName);
      rightWristForceSensorData.packWrench(rightWristWrench);

      rightWristWrench.packLinearPart(rightWristForce);

      rightWristForceBandPassFiltered.update(rightWristForceMagnitude.getDoubleValue());

      if (rightWristForceBandPassFiltered.getDoubleValue() > maxObservedWristForce)
      {
         maxObservedWristForce = rightWristForceBandPassFiltered.getDoubleValue();
      }
   }

   FramePoint2d tempFramePoint2d = new FramePoint2d();

   private void updateCapturePointError()
   {
      tempFramePoint2d.set(yoDesiredCapturePoint.getX(), yoDesiredCapturePoint.getY());

      double error = Math.abs(yoCapturePoint.distance(tempFramePoint2d));

      if (error > maxObservedCapturePointError)
      {
         maxObservedCapturePointError = error;
      }
      capturePointErrorMag.set(error);
   }

   @Override
   public void doControl()
   {
      updateCapturePointError();
      updateWristSensorValues();

      if (capturePointErrorMag.getDoubleValue() > MAX_CAPTURE_POINT_ERROR)
      {
         System.out.println("TurnValveBehavior: MAX CAPTURE POINT ERROR EXCEEDED!  Capture Point Error =  " + capturePointErrorMag.getDoubleValue());
      }

      if (rightWristForceBandPassFiltered.getDoubleValue() > MAX_WRIST_FORCE_THRESHOLD_N)
      {
         System.out.println("TurnValveBehavior: MAX WRIST FORCE EXCEEDED!  Force Magnitude =  " + rightWristForceBandPassFiltered.getDoubleValue());
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
            System.out.println("TurnValveBehavior: Script Resource Stream is null. Can't load script!");
         else
            System.out.println("TurnValveBehavior: Script " + scriptBehaviorInputPacket.getScriptName() + " loaded.");

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
      targetWalkLocation.add(valveOffsetInWorldFrame);
      targetWalkOrientation.setYawPitchRoll(valveOrientation.getYaw().getDoubleValue() + 0.5 * Math.PI, valveOrientation.getPitch().getDoubleValue(),
            valveOrientation.getRoll().getDoubleValue());
      walkToLocationBehavior.setTarget(targetWalkLocation, targetWalkOrientation);

      System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveOffsetInWorldFrame);
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
      System.out.println("TurnValveBehavior: Current Child Behavior: " + currentBehavior.getName());

      System.out.println("TurnValveBehavior: Script Resource Stream" + scriptResourceStream);

      System.out.println("TurnValveBehavior: right wrist Fx : " + rightWristWrench.getLinearPartX());
      System.out.println("TurnValveBehavior: right wrist Fy : " + rightWristWrench.getLinearPartY());
      System.out.println("TurnValveBehavior: right wrist Fz : " + rightWristWrench.getLinearPartZ());

      System.out.println("TurnValveBehavior: max wrist force : " + maxObservedWristForce);
      System.out.println("TurnValveBehavior: max capture point error : " + maxObservedCapturePointError);

      System.out.println("TurnValveBehavior: Valve Location: " + valveLocation);
      System.out.println("TurnValveBehavior: ValveOffset in World Frame: " + valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior: Target Walk to Location: " + targetWalkLocation);
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
