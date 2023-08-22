package us.ihmc.commonWalkingControlModules.controlModules;

import gnu.trove.map.hash.TIntObjectHashMap;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointOfflineCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.List;

public class JointOfflineManager
{
   private final OneDoFJointBasics[] oneDoFJoints;
   private final YoDouble jointOfflineWeight;
   private final TIntObjectHashMap<OneDoFJointBasics> jointHashCodeMap = new TIntObjectHashMap<>();
   private final HashMap<OneDoFJointBasics, YoBoolean> offlineStatus = new HashMap<>();
   private final JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();

   private final SideDependentList<RecyclingArrayList<Point2D>> nominalContactPoints = new SideDependentList<>(side -> new RecyclingArrayList<>(4, Point2D.class));
   private final SideDependentList<MutableBoolean> hasNewContactState = new SideDependentList<>(side -> new MutableBoolean(false));
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final FramePoint3D tempPoint = new FramePoint3D();

   public JointOfflineManager(HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry registry)
   {
      this.oneDoFJoints = controllerToolbox.getControlledOneDoFJoints();
      this.controllerToolbox = controllerToolbox;

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointHashCodeMap.put(oneDoFJoints[i].hashCode(), oneDoFJoints[i]);
         offlineStatus.put(oneDoFJoints[i], new YoBoolean(oneDoFJoints[i].getName() + "IsOffline", registry));
      }

      jointOfflineWeight = new YoDouble("jointOfflineWeight", registry);
      jointOfflineWeight.set(100.0);
   }

   public void handleJointOfflineCommand(JointOfflineCommand jointOfflineCommand)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         offlineStatus.get(oneDoFJoints[i]).set(false);
      }

      jointTorqueCommand.clear();

      for (int i = 0; i < jointOfflineCommand.getNumberOfJointsOffline(); i++)
      {
         OneDoFJointBasics joint = jointHashCodeMap.get(jointOfflineCommand.getJointOfflineHashCode(i));
         if (joint == null)
         {
            throw new RuntimeException("Cannot find joint with hash-code: " + jointOfflineCommand.getJointOfflineHashCode(i));
         }

         if (jointTorqueCommand.getJoints().contains(joint))
         {
            throw new RuntimeException("Joint " + joint.getName() + " occurred multiple times in JointOfflineCommand");
         }

         double desiredJointTorque = 0.0;
         jointTorqueCommand.addJoint(joint, desiredJointTorque);
         offlineStatus.get(joint).set(true);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RecyclingArrayList<Point2D> otherNominalContactPoints = jointOfflineCommand.getNominalFootContactPoints(robotSide);
         if (otherNominalContactPoints.isEmpty())
            continue;

         RecyclingArrayList<Point2D> thisNominalContactPoints = this.nominalContactPoints.get(robotSide);
         thisNominalContactPoints.clear();
         for (int i = 0; i < otherNominalContactPoints.size(); i++)
            thisNominalContactPoints.add().set(otherNominalContactPoints.get(i));

         // For now assume it's always new
         hasNewContactState.get(robotSide).setTrue();
      }
   }

   public void updateFootContactStates()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasNewContactState.get(robotSide).isFalse())
            continue;

         YoPlaneContactState footContactState = controllerToolbox.getFootContactState(robotSide);
         if (!footContactState.inContact()) // only apply when in stance currently
            continue;

         hasNewContactState.get(robotSide).setFalse();

         List<YoContactPoint> contactPoints = footContactState.getContactPoints();
         RecyclingArrayList<Point2D> nominalContactPoints = this.nominalContactPoints.get(robotSide);

         for (int i = 0; i < contactPoints.size(); i++)
         {
            if (i < nominalContactPoints.size())
            {
               contactPoints.get(i).setInContact(true);
               tempPoint.setIncludingFrame(controllerToolbox.getReferenceFrames().getSoleFrame(robotSide), nominalContactPoints.get(i), 0.0);
               contactPoints.get(i).setMatchingFrame(tempPoint);
            }
            else
            {
               contactPoints.get(i).setInContact(false);
            }
         }

         footContactState.notifyContactStateHasChanged();
      }
   }

   public boolean isJointOffline()
   {
      return jointTorqueCommand.getNumberOfJoints() > 0;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      jointTorqueCommand.setWeight(jointOfflineWeight.getDoubleValue());
      return jointTorqueCommand;
   }

}
