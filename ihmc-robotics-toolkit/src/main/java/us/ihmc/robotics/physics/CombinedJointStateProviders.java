package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collector;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public class CombinedJointStateProviders implements JointStateProvider
{
   private final JointStateType state;
   private final List<JointStateProvider> jointStateProviders = new ArrayList<>();
   private final DMatrixRMaj combinedJointState = new DMatrixRMaj(6, 1);

   public CombinedJointStateProviders(JointStateType state)
   {
      this.state = state;
   }

   public CombinedJointStateProviders(CombinedJointStateProviders other)
   {
      this.state = other.state;
      this.jointStateProviders.addAll(other.jointStateProviders);
   }

   public void addAll(CombinedJointStateProviders other)
   {
      if (other.getState() != state)
         throw new IllegalArgumentException("State mismatch: expected " + state + ", was " + other.getState());
      jointStateProviders.addAll(other.jointStateProviders);
   }

   public void addAll(Collection<? extends JointStateProvider> jointStateProviders)
   {
      jointStateProviders.forEach(this::add);
   }

   public void add(JointStateProvider jointStateProvider)
   {
      if (jointStateProvider == null)
         return;

      if (jointStateProvider.getState() != state)
         throw new IllegalArgumentException("State mismatch: expected " + state + ", was " + jointStateProvider.getState());

      jointStateProviders.add(jointStateProvider);
   }

   public void removeAll(CombinedJointStateProviders other)
   {
      jointStateProviders.removeAll(other.jointStateProviders);
   }

   public void removeJointSateProviders(Collection<? extends JointStateProvider> jointStateProviders)
   {
      jointStateProviders.forEach(this::add);
   }

   public void removeJointSateProvider(JointStateProvider jointStateProvider)
   {
      if (jointStateProvider == null)
         return;
      jointStateProviders.remove(jointStateProvider);
   }

   @Override
   public JointStateType getState()
   {
      return state;
   }

   @Override
   public DMatrixRMaj getJointState(JointReadOnly joint)
   {
      combinedJointState.zero();
      combinedJointState.reshape(joint.getDegreesOfFreedom(), 1);

      for (JointStateProvider jointStateProvider : jointStateProviders)
      {
         DMatrixRMaj jointState = jointStateProvider.getJointState(joint);
         if (jointState != null)
            CommonOps_DDRM.addEquals(combinedJointState, jointState);
      }
      return combinedJointState;
   }

   @Override
   public double getJointState(OneDoFJointReadOnly joint)
   {
      double combinedJointState = 0.0;

      for (JointStateProvider jointStateProvider : jointStateProviders)
      {
         double jointState = jointStateProvider.getJointState(joint);
         if (Double.isFinite(jointState))
            combinedJointState += jointState;
      }

      return combinedJointState;
   }

   public static Collector<JointStateProvider, CombinedJointStateProviders, CombinedJointStateProviders> collect(JointStateType state)
   {
      return Collector.of(() -> new CombinedJointStateProviders(state), CombinedJointStateProviders::add, (left, right) ->
      {
         left.addAll(right);
         return left;
      }, Collector.Characteristics.IDENTITY_FINISH);
   }

   public static Collector<ImpulseBasedConstraintCalculator, CombinedJointStateProviders, CombinedJointStateProviders> collectFromCalculator(JointStateType state)
   {
      return Collector.of(() -> new CombinedJointStateProviders(state), (providers, calculator) ->
      {
         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
            providers.add(calculator.getJointTwistChangeProvider(i));
      }, (left, right) ->
      {
         left.addAll(right);
         return left;
      }, Collector.Characteristics.IDENTITY_FINISH);
   }
}
