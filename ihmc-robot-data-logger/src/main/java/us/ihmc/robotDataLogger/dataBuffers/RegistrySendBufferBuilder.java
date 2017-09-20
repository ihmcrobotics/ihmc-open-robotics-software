package us.ihmc.robotDataLogger.dataBuffers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointHolderFactory;
import us.ihmc.robotDataLogger.rtps.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.rtps.DataProducerParticipant;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistrySendBufferBuilder implements us.ihmc.concurrent.Builder<RegistrySendBuffer>
{
   private final YoVariableRegistry registry;
   private final RigidBody rootBody;

   private final List<YoVariable<?>> variables = new ArrayList<>();
   private final List<JointHolder> jointHolders = new ArrayList<>();

   private final LoggerDebugRegistry loggerDebugRegistry;

   private final YoGraphicsListRegistry graphics;

   private int registryID = -1;

   public RegistrySendBufferBuilder(YoVariableRegistry registry, RigidBody rootBody, YoGraphicsListRegistry graphics)
   {
      this.registry = registry;
      this.rootBody = rootBody;
      this.graphics = graphics;

      this.loggerDebugRegistry = new LoggerDebugRegistry(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public List<YoVariable<?>> getVariables()
   {
      return variables;
   }

   public void build(int registryID)
   {
      this.registryID = registryID;

      if (rootBody != null)
      {
         InverseDynamicsJoint[] joints = ScrewTools.computeSubtreeJoints(rootBody);
         for (InverseDynamicsJoint joint : joints)
         {
            JointHolder jointHolder = JointHolderFactory.getJointHolder(joint);
            jointHolders.add(jointHolder);
         }

         int numberOfJointStates = getNumberOfJointStates();
         if (CustomLogDataPublisherType.getTypeSize(0, numberOfJointStates) > DataProducerParticipant.getMaximumSynchronousPacketSize())
         {
            throw new RuntimeException("Exceeded maximum number of joint states (Approx. " + ((DataProducerParticipant.getMaximumSynchronousPacketSize() - CustomLogDataPublisherType.getTypeSize(0, 0)) / 8 - 1) + ")");
         }
      }

   }

   public List<JointHolder> getJointHolders()
   {
      return jointHolders;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return graphics;
   }

   @Override
   public RegistrySendBuffer newInstance()
   {
      if (registryID == -1)
      {
         throw new RuntimeException("RegistrySendBufferBuilder.build() not called");
      }

      if (variables.size() == 0)
      {
         throw new RuntimeException("Variables not populated");
      }

      return new RegistrySendBuffer(registryID, variables, jointHolders);
   }

   public int getRegistryID()
   {
      return registryID;
   }

   public int getNumberOfJointStates()
   {
      return getNumberOfJointStates(jointHolders);
   }

   public int getNumberOfVariables()
   {
      return variables.size();
   }

   public static int getNumberOfJointStates(List<JointHolder> jointHolders)
   {
      int numberOfJointStates = 0;
      for (int i = 0; i < jointHolders.size(); i++)
      {
         numberOfJointStates += jointHolders.get(i).getNumberOfStateVariables();
      }
      return numberOfJointStates;
   }

   public LoggerDebugRegistry getLoggerDebugRegistry()
   {
      return loggerDebugRegistry;
   }
}