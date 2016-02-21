package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Queue;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.controllers.SimplePDGainsHolder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceFeedbackControlCommand extends FeedbackControlCommand<JointspaceFeedbackControlCommand>
{
   private final int initialCapacity = 15;
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final Queue<MutableInt> unusedIntegers = new ArrayDeque<>(initialCapacity);
   private final LinkedHashMap<InverseDynamicsJoint, MutableInt> jointToJointIndexMap = new LinkedHashMap<>(initialCapacity);

   private final RecyclingArrayList<MutableDouble> desiredPositions = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> desiredVelocities = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> feedForwardAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   private final SimplePDGainsHolder gains = new SimplePDGainsHolder();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public JointspaceFeedbackControlCommand()
   {
      super(FeedbackControlCommandType.JOINTSPACE_CONTROL);
      initializeUnusedIntegers();
      clear();
   }

   private void initializeUnusedIntegers()
   {
      for (int i = 0; i < initialCapacity; i++)
         unusedIntegers.add(new MutableInt(-1));
   }

   public void clear()
   {
      while (!joints.isEmpty())
         unusedIntegers.add(jointToJointIndexMap.remove(joints.remove(joints.size() - 1)));

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
      desiredPositions.add().setValue(desiredPosition);
      desiredVelocities.add().setValue(desiredVelocity);
      feedForwardAccelerations.add().setValue(feedForwardAcceleration);

      updateIndexMap(joint, joints.size() - 1);
   }

   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
   }

   public void setOneDoFJointDesiredAcceleration(int jointIndex, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      MathTools.checkIfEqual(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredPositions.get(jointIndex).setValue(desiredPosition);
      desiredVelocities.get(jointIndex).setValue(desiredVelocity);
      feedForwardAccelerations.get(jointIndex).setValue(feedForwardAcceleration);
   }

   public void setOneDoFJointDesiredAcceleration(OneDoFJoint joint, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      int jointIndex = jointToJointIndexMap.get(joint).intValue();
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
         desiredPositions.add().setValue(other.getDesiredPosition(i));
         desiredVelocities.add().setValue(other.getDesiredVelocity(i));
         feedForwardAccelerations.add().setValue(other.getFeedForwardAcceleration(i));
      }
   }

   private void updateIndexMap(InverseDynamicsJoint joint, int jointIndex)
   {
      if (unusedIntegers.isEmpty())
      {
         jointToJointIndexMap.put(joint, new MutableInt(joints.size() - 1));
      }
      else
      {
         MutableInt jointMutableIndex = unusedIntegers.remove();
         jointMutableIndex.setValue(jointIndex);
         jointToJointIndexMap.put(joint, jointMutableIndex);
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
