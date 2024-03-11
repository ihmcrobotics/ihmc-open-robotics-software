package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoSpatialInertia;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;

public class InertialBaselineCalculator
{
   private final Set<SpatialInertiaBasisOption>[] basisSets;
   private final SpatialInertiaReadOnly[] urdfSpatialInertias;
   private final YoSpatialInertia[] tareSpatialInertias;

   private final YoMatrix[] parameterDeltas;
   private final RateLimitedYoVariable[][] rateLimitedParameterDeltas;
   private final DMatrixRMaj rateLimitedParameterDeltaContainer;

   public InertialBaselineCalculator(FullHumanoidRobotModel model, InertialEstimationParameters parameters, double dt, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      basisSets = parameters.getBasisSets();

      int nBodies = model.getRootBody().subtreeArray().length;
      tareSpatialInertias = new YoSpatialInertia[nBodies];
      urdfSpatialInertias = new SpatialInertiaBasics[nBodies];
      RigidBodyBasics[] bodies = model.getRootBody().subtreeArray();
      for (int i = 0; i < nBodies; i++)
      {
         tareSpatialInertias[i] = new YoSpatialInertia(bodies[i].getInertia(), "_tare", registry);
         urdfSpatialInertias[i] = new SpatialInertia(bodies[i].getInertia());
      }

      String[] basisNames = RigidBodyInertialParametersTools.getNamesForPiBasis();
      double[] defaultMaxParameterDeltaRates = parameters.getMaxParameterDeltaRates();
      YoDouble[] maxParameterDeltaRates = new YoDouble[RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      for (int i = 0; i < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; i++)
      {
         maxParameterDeltaRates[i] = new YoDouble("maxParameterDeltaRate_" + basisNames[i], registry);
         maxParameterDeltaRates[i].set(defaultMaxParameterDeltaRates[i]);
      }
      parameterDeltas = new YoMatrix[nBodies];
      rateLimitedParameterDeltas = new RateLimitedYoVariable[nBodies][RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY];
      for (int i = 0; i < nBodies; i++)
      {
         parameterDeltas[i] = new YoMatrix("parameterDelta_" + bodies[i].getName(),
                                           RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY,
                                           1,
                                           RigidBodyInertialParametersTools.getNamesForPiBasis(),
                                           registry);
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
         {
            rateLimitedParameterDeltas[i][j] = new RateLimitedYoVariable("rateLimitedParameterDelta_" + bodies[i].getName() + "_" + basisNames[j],
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
    * TODO: doc me
    * @param bodies
    */
   public void updateTareSpatialInertias(RigidBodyBasics[] bodies)
   {
      if (bodies.length != tareSpatialInertias.length)
         throw new RuntimeException("The number of bodies does not match the number of tare spatial inertias.");

      for (int i = 0; i < bodies.length; i++)
      {
         if (!basisSets[i].isEmpty())  // Only tare the bodies we're estimating
            continue;

         tareSpatialInertias[i].set(bodies[i].getInertia());
      }
   }

   /**
    * TODO: doc me
    * @param bodies
    */
   public void calculateRateLimitedParameterDeltas(RigidBodyReadOnly[] bodies)
   {
      for (int i = 0; i < bodies.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only update the bodies we're estimating
            continue;

         RigidBodyInertialParametersTools.calculateParameterDelta(bodies[i].getInertia(), tareSpatialInertias[i], parameterDeltas[i]);
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
            rateLimitedParameterDeltas[i][j].update();
      }
   }

   /**
    * TODO: doc me
    * @param bodies
    */
   public void addRateLimitedParameterDeltas(RigidBodyBasics[] bodies)
   {
      for (int i = 0; i < bodies.length; i++)
      {
         if (basisSets[i].isEmpty())  // Only update the bodies we're estimating
            continue;

         // Add the deltas from the tare value to the urdf value -- these are what we send to the controller.
         for (int j = 0; j < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; j++)
            rateLimitedParameterDeltaContainer.set(j, 0, rateLimitedParameterDeltas[i][j].getDoubleValue());
         RigidBodyInertialParametersTools.addParameterDelta(urdfSpatialInertias[i], rateLimitedParameterDeltaContainer, bodies[i].getInertia());
      }
   }
}
