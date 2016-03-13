package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.controllers.SimplePDGainsHolder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceFeedbackControlCommand implements FeedbackControlCommand<JointspaceFeedbackControlCommand>
{
   private final int initialCapacity = 15;
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final List<String> jointNames = new ArrayList<>(initialCapacity);

   private final RecyclingArrayList<MutableDouble> desiredPositions = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> desiredVelocities = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> feedForwardAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   private final SimplePDGainsHolder gains = new SimplePDGainsHolder();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public JointspaceFeedbackControlCommand()
   {
      clear();
   }

   public void clear()
   {
      joints.clear();
      jointNames.clear();
      desiredPositions.clear();
      desiredVelocities.clear();
      feedForwardAccelerations.clear();
   }

   public void setGains(PDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   public void addJoint(OneDoFJoint joint, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredPositions.add().setValue(desiredPosition);
      desiredVelocities.add().setValue(desiredVelocity);
      feedForwardAccelerations.add().setValue(feedForwardAcceleration);
   }

   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
   }

   public void setOneDoFJoint(int jointIndex, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      MathTools.checkIfEqual(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredPositions.get(jointIndex).setValue(desiredPosition);
      desiredVelocities.get(jointIndex).setValue(desiredVelocity);
      feedForwardAccelerations.get(jointIndex).setValue(feedForwardAcceleration);
   }

   @Override
   public void set(JointspaceFeedbackControlCommand other)
   {
      clear();
      gains.set(other.gains);
      weightForSolver = other.weightForSolver;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
         desiredPositions.add().setValue(other.getDesiredPosition(i));
         desiredVelocities.add().setValue(other.getDesiredVelocity(i));
         feedForwardAccelerations.add().setValue(other.getFeedForwardAcceleration(i));
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         joints.set(i, nameToJointMap.get(jointNames.get(i)));
      }
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public PDGainsInterface getGains()
   {
      return gains;
   }

   public OneDoFJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   public double getDesiredPosition(int jointIndex)
   {
      return desiredPositions.get(jointIndex).doubleValue();
   }

   public double getDesiredVelocity(int jointIndex)
   {
      return desiredVelocities.get(jointIndex).doubleValue();
   }

   public double getFeedForwardAcceleration(int jointIndex)
   {
      return feedForwardAccelerations.get(jointIndex).doubleValue();
   }

   public double getWeightForSolver()
   {
      return weightForSolver;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   public String toString()
   {
      String ret = getClass().getSimpleName() + ": ";
      for (int i = 0; i < joints.size(); i++)
      {
         ret += joints.get(i).getName();
         if (i < joints.size() - 1)
            ret += ", ";
         else
            ret += ".";
      }
      return ret;
   }
}
