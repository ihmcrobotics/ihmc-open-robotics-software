package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ReachingManifoldCommand
      implements Command<ReachingManifoldCommand, ReachingManifoldMessage>, WholeBodyTrajectoryToolboxAPI<ReachingManifoldMessage>
{
   private long sequenceId;
   private int rigidBodyHashCode;
   private RigidBodyBasics rigidBody;

   private Point3D manifoldOriginPosition;
   private Quaternion manifoldOriginOrientation;

   private List<ConfigurationSpaceName> manifoldConfigurationSpaces = new ArrayList<>();
   private TDoubleArrayList manifoldLowerLimits = new TDoubleArrayList();
   private TDoubleArrayList manifoldUpperLimits = new TDoubleArrayList();

   public ReachingManifoldCommand()
   {

   }

   public ReachingManifoldCommand(RigidBodyBasics rigidBody, Point3D manifoldOriginPosition, Quaternion manifoldOriginOrientation,
                                  ConfigurationSpaceName... configurationSpaces)
   {
      clear();
      this.rigidBody = rigidBody;
      this.rigidBodyHashCode = rigidBody.hashCode();
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

      sequenceId = other.sequenceId;

      rigidBodyHashCode = other.rigidBodyHashCode;
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
   public void setFromMessage(ReachingManifoldMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(ReachingManifoldMessage message, Map<Integer, RigidBodyBasics> rigidBodyHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      sequenceId = message.getSequenceId();
      rigidBodyHashCode = message.getEndEffectorHashCode();
      if (rigidBodyHashMap == null)
         rigidBody = null;
      else
         rigidBody = rigidBodyHashMap.get(rigidBodyHashCode);

      this.manifoldOriginPosition.set(message.getManifoldOriginPosition());
      this.manifoldOriginOrientation.set(message.getManifoldOriginOrientation());

      for (int i = 0; i < message.getManifoldConfigurationSpaceNames().size(); i++)
      {
         manifoldConfigurationSpaces.add(ConfigurationSpaceName.fromByte(message.getManifoldConfigurationSpaceNames().get(i)));
         manifoldLowerLimits.add(message.getManifoldLowerLimits().get(i));
         manifoldUpperLimits.add(message.getManifoldUpperLimits().get(i));
      }
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      rigidBodyHashCode = 0;
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
      return !manifoldOriginPosition.containsNaN() && !manifoldOriginOrientation.containsNaN();
   }

   public RigidBodyBasics getRigidBody()
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

   public TDoubleArrayList getManifoldLowerLimits()
   {
      return manifoldLowerLimits;
   }

   public TDoubleArrayList getManifoldUpperLimits()
   {
      return manifoldUpperLimits;
   }

   public Point3D getManifoldOriginPosition()
   {
      return manifoldOriginPosition;
   }

   public Quaternion getManifoldOriginOrientation()
   {
      return manifoldOriginOrientation;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
