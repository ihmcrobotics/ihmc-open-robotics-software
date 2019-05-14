package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextData implements InPlaceCopyable<HumanoidRobotContextData>
{
   /** Serves to synchronize the controller time to the estimator time. The estimator sets this, the controller reads it. */
   private long timestamp = Long.MIN_VALUE;
   /** Serves to inform the estimator that the controller ran and populated the desired values in this context. Set by the controller. */
   private boolean controllerRan = false;
   /** Serves to inform the controller that the estimator ran and populated the estimated values in this context. Set by the estimator. */
   private boolean estimatorRan = false;

   private final HumanoidRobotContextJointData processedJointData;

   private final ForceSensorDataHolder forceSensorDataHolder;
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final RobotMotionStatusHolder robotMotionStatusHolder;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final ArrayList<RigidBodyBasics> robotFeet;
   private final Map<String, FramePoint2D> copPoints = new HashMap<>();

   protected HumanoidRobotContextData(HumanoidRobotContextJointData processedJointData, ForceSensorDataHolder forceSensorDataHolder,
                                      CenterOfPressureDataHolder centerOfPressureDataHolder, RobotMotionStatusHolder robotMotionStatusHolder,
                                      JointDesiredOutputList jointDesiredOutputList)
   {
      this.processedJointData = processedJointData;
      this.forceSensorDataHolder = forceSensorDataHolder;

      this.robotFeet = new ArrayList<>();
      for (int i = 0; i < centerOfPressureDataHolder.getNumberOfBodiesWithCenterOfPressure(); i++)
      {
         robotFeet.add(centerOfPressureDataHolder.getRigidBody(i));
      }

      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.jointDesiredOutputList = jointDesiredOutputList;

      for (int i = 0; i < robotFeet.size(); i++)
      {
         RigidBodyBasics foot = robotFeet.get(i);
         copPoints.put(foot.getName(), new FramePoint2D());
      }
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      this.timestamp = src.timestamp;

      this.processedJointData.copyFrom(src.processedJointData);

      this.forceSensorDataHolder.set(src.forceSensorDataHolder);

      for (int i = 0; i < robotFeet.size(); i++)
      {
         RigidBodyBasics foot = robotFeet.get(i);
         FramePoint2D copTmp = copPoints.get(foot.getName());

         src.centerOfPressureDataHolder.getCenterOfPressure(copTmp, foot);
         this.centerOfPressureDataHolder.setCenterOfPressure(copTmp, foot);
      }

      this.robotMotionStatusHolder.setCurrentRobotMotionStatus(src.robotMotionStatusHolder.getCurrentRobotMotionStatus());

      for (int i = 0; i < this.jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         this.jointDesiredOutputList.getJointDesiredOutput(i).set(src.jointDesiredOutputList.getJointDesiredOutput(i));
      }
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public void setControllerRan(boolean controllerRan)
   {
      this.controllerRan = controllerRan;
   }

   public boolean getControllerRan()
   {
      return controllerRan;
   }

   public void setEstimatorRan(boolean estimatorRan)
   {
      this.estimatorRan = estimatorRan;
   }

   public boolean getEstimatorRan()
   {
      return estimatorRan;
   }
}
