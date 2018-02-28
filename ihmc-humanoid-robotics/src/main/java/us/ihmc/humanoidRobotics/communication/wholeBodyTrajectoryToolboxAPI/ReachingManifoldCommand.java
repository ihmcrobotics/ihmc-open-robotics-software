package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ReachingManifoldCommand
      implements Command<ReachingManifoldCommand, ReachingManifoldMessage>, WholeBodyTrajectoryToolboxAPI<ReachingManifoldMessage>
{
   private long rigidBodyNameBasedashCode;
   private RigidBody rigidBody;

   private Point3D manifoldOriginPosition;
   private Quaternion manifoldOriginOrientation;

   private List<ConfigurationSpaceName> manifoldConfigurationSpaces = new ArrayList<>();
   private TDoubleArrayList manifoldLowerLimits = new TDoubleArrayList();
   private TDoubleArrayList manifoldUpperLimits = new TDoubleArrayList();

   public ReachingManifoldCommand()
   {

   }

   public ReachingManifoldCommand(RigidBody rigidBody, Point3D manifoldOriginPosition, Quaternion manifoldOriginOrientation,
                                  ConfigurationSpaceName... configurationSpaces)
   {
      clear();
      this.rigidBody = rigidBody;
      this.rigidBodyNameBasedashCode = rigidBody.getNameBasedHashCode();
      this.manifoldOriginPosition.set(manifoldOriginPosition);
      this.manifoldOriginOrientation.set(manifoldOriginOrientation);
      for (int i = 0; i < configurationSpaces.length; i++)
         this.manifoldConfigurationSpaces.add(configurationSpaces[i]);
      this.manifoldLowerLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationLowerLimitArray(configurationSpaces));
      this.manifoldUpperLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationUpperLimitArray(configurationSpaces));
   }

   @Override
   public void set(ReachingManifoldCommand other)
   {
      clear();

      rigidBodyNameBasedashCode = other.rigidBodyNameBasedashCode;
      rigidBody = other.rigidBody;

      this.manifoldOriginPosition.set(other.manifoldOriginPosition);
      this.manifoldOriginOrientation.set(other.manifoldOriginOrientation);

      for (int i = 0; i < other.getDimensionOfManifold(); i++)
      {
         manifoldConfigurationSpaces.add(other.manifoldConfigurationSpaces.get(i));
         manifoldLowerLimits.add(other.manifoldLowerLimits.get(i));
         manifoldUpperLimits.add(other.manifoldUpperLimits.get(i));
      }
   }

   @Override
   public void set(ReachingManifoldMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(ReachingManifoldMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      rigidBodyNameBasedashCode = message.getRigidBodyNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         rigidBody = null;
      else
         rigidBody = rigidBodyNamedBasedHashMap.get(rigidBodyNameBasedashCode);

      this.manifoldOriginPosition.set(message.getOriginPosition());
      this.manifoldOriginOrientation.set(message.getOriginOrientation());

      for (int i = 0; i < message.getDimensionOfManifold(); i++)
      {
         manifoldConfigurationSpaces.add(ConfigurationSpaceName.fromByte(message.getDegreeOfManifold(i)));
         manifoldLowerLimits.add(message.getLowerLimit(i));
         manifoldUpperLimits.add(message.getUpperLimit(i));
      }
   }

   @Override
   public void clear()
   {
      rigidBodyNameBasedashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      rigidBody = null;
      manifoldOriginPosition = new Point3D();
      manifoldOriginOrientation = new Quaternion();
      manifoldConfigurationSpaces.clear();
      manifoldLowerLimits.reset();
      manifoldUpperLimits.reset();
   }

   @Override
   public Class<ReachingManifoldMessage> getMessageClass()
   {
      return ReachingManifoldMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return false;
   }

   public Pose3D computeClosestPoseOnManifold(Pose3D pose)
   {
      double positionWeight = 1.0;
      double orientationWeight = 0.0;
      double closestDistance = Double.MAX_VALUE;
      double distanceOld = closestDistance;

      double alpha = -7.0;
      double perturb = 0.01;

      int maximumNumberOfIteration = 100;

      TDoubleArrayList closestConfigurationSpace = new TDoubleArrayList();

      // initial : medium values
      for (int i = 0; i < getDimensionOfManifold(); i++)
      {
         closestConfigurationSpace.add((getUpperLimit(i) + getLowerLimit(i)) / 2);
      }

      // start iteration
      for (int i = 0; i < maximumNumberOfIteration; i++)
      {
         // get pose
         Pose3D closestPoseControl = computePoseOnManifold(closestConfigurationSpace);

         // closest distance 
         closestDistance = WholeBodyTrajectoryToolboxMessageTools.computePoseDistance(pose, closestPoseControl, positionWeight, orientationWeight);

         // gradient decent
         TDoubleArrayList gradientDecent = new TDoubleArrayList();
         for (int j = 0; j < getDimensionOfManifold(); j++)
         {
            TDoubleArrayList perturbedConfigurationSpace = new TDoubleArrayList(closestConfigurationSpace);
            perturbedConfigurationSpace.set(j, perturbedConfigurationSpace.get(j) + perturb);

            perturbedConfigurationSpace.set(j, MathTools.clamp(perturbedConfigurationSpace.get(j), getLowerLimit(j), getUpperLimit(j)));

            Pose3D perturbedPoseControl = computePoseOnManifold(perturbedConfigurationSpace);
            double perturbedDistance = WholeBodyTrajectoryToolboxMessageTools.computePoseDistance(pose, perturbedPoseControl, positionWeight,
                                                                                                  orientationWeight);
            gradientDecent.add((perturbedDistance - closestDistance) / perturb);
         }

         // step alpha
         TDoubleArrayList currentConfigurationSpace = new TDoubleArrayList();
         for (int j = 0; j < getDimensionOfManifold(); j++)
         {
            currentConfigurationSpace.add(closestConfigurationSpace.get(j) + alpha * gradientDecent.get(j));

            currentConfigurationSpace.set(j, MathTools.clamp(currentConfigurationSpace.get(j), getLowerLimit(j), getUpperLimit(j)));
         }

         Pose3D currentPoseControl = computePoseOnManifold(currentConfigurationSpace);

         double currentDistance = WholeBodyTrajectoryToolboxMessageTools.computePoseDistance(pose, currentPoseControl, positionWeight, orientationWeight);

         if (currentDistance < closestDistance)
         {
            closestDistance = currentDistance;
            closestConfigurationSpace = new TDoubleArrayList(currentConfigurationSpace);
         }
         else
         {
            return computePoseOnManifold(closestConfigurationSpace);
         }
      }

      return computePoseOnManifold(closestConfigurationSpace);
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public int getDimensionOfManifold()
   {
      return manifoldConfigurationSpaces.size();
   }

   public ConfigurationSpaceName getDegreeOfManifold(int i)
   {
      return manifoldConfigurationSpaces.get(i);
   }

   public double getUpperLimit(int i)
   {
      return manifoldUpperLimits.get(i);
   }

   public double getLowerLimit(int i)
   {
      return manifoldLowerLimits.get(i);
   }

   public Pose3D computePoseOnManifold(TDoubleArrayList configurationSpace)
   {
      Pose3D pose = new Pose3D(manifoldOriginPosition, manifoldOriginOrientation);

      for (int i = 0; i < manifoldConfigurationSpaces.size(); i++)
      {
         pose.appendTransform(manifoldConfigurationSpaces.get(i).getLocalRigidBodyTransform(configurationSpace.get(i)));
      }

      return pose;
   }
}
