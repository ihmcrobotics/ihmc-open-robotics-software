package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;

/**
 * This class is used to calculate the differences ("deltas") in inertial parameters between a set of rigid bodies and the nominal (URDF) values, as well as
 * adding the calculated deltas to another set of rigid bodies actually used by the controller.
 * <p>
 * The deltas are fed to the controller's rigid bodies in a rate-limited manner to avoid large jumps in the inertial parameters.
 * </p>
 * <p>
 * There used to be a separate, operator-movable "tare" that the deltas were calculated FROM, distinct from the nominal values they were added TO. It was
 * removed: moving it broke the convex-interpolation property relied on below (see {@link #addRateLimitedParameterDeltas}), and it duplicated -- and could
 * silently contradict -- the estimator's own notion of nominal (theta = 0 in the generalized parameterization). Model mismatch on the real robot is anchored in
 * TORQUE space instead, by the measurement bias; see {@code tareProcess} in {@link InertialParameterManager}.
 * </p>
 *
 * @author James Foster
 */
public class InertialBaselineCalculator
{
   private final Set<SpatialInertiaBasisOption>[] basisSets;
   /** The nominal (URDF) spatial inertias: both what the parameter deltas are calculated FROM and what they are added TO. */
   private final SpatialInertiaReadOnly[] urdfSpatialInertias;

   private final YoMatrix[] parameterDeltas;
   private final RateLimitedYoVariable[][] rateLimitedParameterDeltas;
   private final DMatrixRMaj rateLimitedParameterDeltaContainer;

   public InertialBaselineCalculator(FullHumanoidRobotModel model, InertialEstimationParameters parameters, double dt, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      basisSets = parameters.getBasisSets();

      int nBodies = model.getRootBody().subtreeArray().length;
      urdfSpatialInertias = new SpatialInertiaBasics[nBodies];
      RigidBodyBasics[] bodies = model.getRootBody().subtreeArray();
      for (int i = 0; i < nBodies; i++)
         urdfSpatialInertias[i] = new SpatialInertia(bodies[i].getInertia());

      String[] basisNames = RigidBodyInertialParametersTools.getNamesForPiBasis();
      double[] defaultMaxParameterDeltaRates = parameters.getMaxParameterDeltaRates();
      YoDouble[] maxParameterDeltaRates = new YoDouble[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      for (int i = 0; i < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; i++)
      {
         maxParameterDeltaRates[i] = new YoDouble("maxDeltaRate_" + basisNames[i], registry);
         maxParameterDeltaRates[i].set(defaultMaxParameterDeltaRates[i]);
      }
      parameterDeltas = new YoMatrix[nBodies];
      rateLimitedParameterDeltas = new RateLimitedYoVariable[nBodies][RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      for (int i = 0; i < nBodies; i++)
      {
         if (basisSets[i].isEmpty())  // Only create objects for bodies we're estimating
            continue;

         parameterDeltas[i] = new YoMatrix("delta_" + bodies[i].getName() + "_",
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           1,
                                           RigidBodyInertialParametersTools.getNamesForPiBasis(),
                                           registry);
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
         {
            rateLimitedParameterDeltas[i][j] = new RateLimitedYoVariable("rateLimitedDelta_" + bodies[i].getName() + "_" + basisNames[j],
                                                                         registry,
                                                                         maxParameterDeltaRates[j],
                                                                         parameterDeltas[i].getYoDouble(j, 0),
                                                                         dt);
            rateLimitedParameterDeltas[i][j].update();
         }
      }
      rateLimitedParameterDeltaContainer = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
   }

   /**
    * Calculate the rate-limited inertial parameter deltas from nominal for the bodies that are being estimated.
    *
    * @param bodies the list of bodies to calculate the rate-limited inertial parameter deltas from nominal for. Not modified.
    */
   public void calculateRateLimitedParameterDeltas(RigidBodyReadOnly[] bodies)
   {
      for (int i = 0; i < bodies.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only update the bodies we're estimating
            continue;

         RigidBodyInertialParametersTools.calculateParameterDelta(bodies[i].getInertia(), urdfSpatialInertias[i], parameterDeltas[i]);
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
            rateLimitedParameterDeltas[i][j].update();
      }
   }

   /**
    * Add the rate-limited inertial parameter deltas to the URDF spatial inertias, scaled by a trust gain.
    * <p>
    * The delta is calculated from the SAME nominal values it is added back to, so the result per body is
    * <pre>
    *   nominal + gain * (estimate - nominal)  =  (1 - gain) * nominal + gain * estimate
    * </pre>
    * i.e. a true convex combination of the nominal (URDF) values ({@code gain = 0}) and the full rate-limited
    * estimate ({@code gain = 1}). Because the physically-consistent inertial parameters form a convex cone and
    * both endpoints lie in it, any {@code gain} in [0, 1] also yields a physically-consistent result; values
    * outside [0, 1] extrapolate and lose that guarantee.
    * <p>
    * This property is why the movable "tare" was removed. With a tare distinct from the nominal, the result was
    * {@code nominal + gain * (estimate - tare)} -- a translation, not an interpolation. Cones are not closed under
    * subtraction, so that expression could leave the physically-consistent set entirely (e.g. a tare snapshotted
    * from a diverged estimate could drive the mass negative).
    *
    * @param bodies the list of bodies to add and pack the inertial parameter deltas into. Modified.
    * @param gain   trust factor scaling the applied delta (0 = nominal, 1 = full estimate).
    */
   public void addRateLimitedParameterDeltas(RigidBodyBasics[] bodies, double gain)
   {
      for (int i = 0; i < bodies.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only update the bodies we're estimating
            continue;

         // Add the (gain-scaled) deltas from the tare value to the urdf value -- these are what we send to the controller.
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
            rateLimitedParameterDeltaContainer.set(j, 0, gain * rateLimitedParameterDeltas[i][j].getDoubleValue());
         RigidBodyInertialParametersTools.addParameterDelta(urdfSpatialInertias[i], rateLimitedParameterDeltaContainer, bodies[i].getInertia());
      }
   }

   /**
    * Zeroes the parameter deltas and clears the rate limiters' memory, so that nothing of a previous (possibly
    * diverged) estimate is left to slew back down from.
    */
   public void resetParameterDeltas()
   {
      for (int i = 0; i < parameterDeltas.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only the bodies we're estimating have deltas
            continue;

         parameterDeltas[i].zero();
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
         {
            rateLimitedParameterDeltas[i][j].set(0.0);
            rateLimitedParameterDeltas[i][j].reset();
         }
      }
   }

   /**
    * Restores the inertial parameters of the bodies being estimated back to their nominal (URDF) values, discarding
    * any deltas previously added to them.
    *
    * @param bodies the list of bodies to restore to nominal. Modified.
    */
   public void restoreNominalParameters(RigidBodyBasics[] bodies)
   {
      rateLimitedParameterDeltaContainer.zero();
      for (int i = 0; i < bodies.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only restore the bodies we're estimating
            continue;

         // A zero delta on top of the nominal is the nominal -- reused so the frame handling matches the normal path.
         RigidBodyInertialParametersTools.addParameterDelta(urdfSpatialInertias[i], rateLimitedParameterDeltaContainer, bodies[i].getInertia());
      }
   }

   public SpatialInertiaReadOnly[] getURDFSpatialInertias()
   {
      return urdfSpatialInertias;
   }
}