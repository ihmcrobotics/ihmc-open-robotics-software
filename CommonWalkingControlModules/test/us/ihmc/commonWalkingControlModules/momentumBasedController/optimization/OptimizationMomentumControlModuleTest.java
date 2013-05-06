package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;
import us.ihmc.utilities.test.JUnitTools;

import javax.vecmath.Vector3d;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

import static junit.framework.Assert.assertTrue;

/**
 * @author twan
 *         Date: 5/4/13
 */
public class OptimizationMomentumControlModuleTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   @Test
   public void test()
   {
      Random random = new Random(1223525L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, Z, Z, X, Y, X, Y
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      InverseDynamicsJoint rootJoint = randomFloatingChain.getRootJoint();
      double totalMass = TotalMassCalculator.computeSubTreeMass(randomFloatingChain.getElevator());

      double controlDT = 5e-3;
      double gravityZ = 9.81;
      double coefficientOfFriction = 1.0;
      YoVariableRegistry registry = new YoVariableRegistry("test");
      MomentumOptimizationSettings momentumOptimizationSettings = createStandardOptimizationSettings(registry);

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      InverseDynamicsJoint[] jointsToOptimizeFor = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, registry,
                                                                   jointsToOptimizeFor, momentumOptimizationSettings, gravityZ);


      RigidBody endEffector = randomFloatingChain.getLeafBody();
      ReferenceFrame soleFrame = endEffector.getBodyFixedFrame();
      ContactablePlaneBody contactablePlaneBody = new RectangularContactableBody(endEffector, soleFrame, 1.0, -1.0, 1.0, -1.0);
      LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
      YoPlaneContactState contactState = new YoPlaneContactState("testContactState", endEffector.getParentJoint().getFrameAfterJoint(), soleFrame, registry);
      contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);
      contactStates.put(contactablePlaneBody, contactState);

      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      SpatialForceVector momentumRateOfChangeIn = generateRandomFeasibleMomentumRateOfChange(centerOfMassFrame, contactStates, totalMass, gravityZ, random,
                                                     momentumOptimizationSettings.getRhoMinScalar());
      momentumRateOfChangeData.set(momentumRateOfChangeIn);


      RobotSide upcomingSupportLeg = null;
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
      momentumControlModule.compute(contactStates, upcomingSupportLeg);
      SpatialForceVector momentumRateOfChangeOut = momentumControlModule.getDesiredCentroidalMomentumRate();

      assertWrenchesSumUpToMomentumDot(momentumControlModule.getExternalWrenches(), momentumRateOfChangeOut, gravityZ, totalMass, centerOfMassFrame);
      assertWrenchesInFrictionCones(momentumControlModule.getExternalWrenches(), contactStates, coefficientOfFriction);

      DenseMatrix64F vd = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(jointsToOptimizeFor, vd);

      double epsilon = 1e-3;
      JUnitTools.assertSpatialForceVectorEquals(momentumRateOfChangeIn, momentumRateOfChangeOut, epsilon);

   }

   private SpatialForceVector generateRandomFeasibleMomentumRateOfChange(ReferenceFrame centerOfMassFrame,
           LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates, double totalMass, double gravityZ, Random random, double rhoMin)

   {
      ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator = new ContactPointWrenchMatrixCalculator(centerOfMassFrame,
                                                                                 MomentumOptimizerNative.nSupportVectors, MomentumOptimizerNative.rhoSize);
      contactPointWrenchMatrixCalculator.computeMatrix(contactStates.values());
      DenseMatrix64F rho = new DenseMatrix64F(MomentumOptimizerNative.rhoSize, 1);
      RandomMatrices.setRandom(rho, random);
      CommonOps.add(rho, rhoMin);
      contactPointWrenchMatrixCalculator.computeWrenches(contactStates.values(), rho);
      Wrench ret = TotalWrenchCalculator.computeTotalWrench(contactPointWrenchMatrixCalculator.getWrenches().values(), centerOfMassFrame);
      ret.setLinearPartZ(ret.getLinearPartZ() - totalMass * gravityZ);

      return ret;
   }

   private static MomentumOptimizationSettings createStandardOptimizationSettings(YoVariableRegistry registry)
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(registry);
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 1.0, 1.0);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(1e-11);
      momentumOptimizationSettings.setGroundReactionForceRegularization(1e-9);
      momentumOptimizationSettings.setRhoMin(0.0);

      return momentumOptimizationSettings;
   }

   private void assertWrenchesSumUpToMomentumDot(Map<ContactablePlaneBody, Wrench> externalWrenches, SpatialForceVector desiredCentroidalMomentumRate,
           double gravityZ, double mass, ReferenceFrame centerOfMassFrame)
   {
      SpatialForceVector totalWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);
      for (Wrench wrench : externalWrenches.values())
      {
         wrench.changeBodyFrameAttachedToSameBody(centerOfMassFrame);
         wrench.changeFrame(centerOfMassFrame);
         totalWrench.add(wrench);
      }

      Wrench gravitationalWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);
      gravitationalWrench.setLinearPartZ(-mass * gravityZ);
      totalWrench.add(gravitationalWrench);

      JUnitTools.assertSpatialForceVectorEquals(desiredCentroidalMomentumRate, totalWrench, 1e-3);
   }

   private void assertWrenchesInFrictionCones(Map<ContactablePlaneBody, Wrench> externalWrenches,
           LinkedHashMap<ContactablePlaneBody, PlaneContactState> contactStates, double coefficientOfFriction)
   {
      CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

      for (ContactablePlaneBody contactablePlaneBody : externalWrenches.keySet())
      {
         Wrench wrench = externalWrenches.get(contactablePlaneBody);
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         ReferenceFrame planeFrame = contactState.getPlaneFrame();

         wrench.changeFrame(planeFrame);

         double fZ = wrench.getLinearPartZ();
         assertTrue(fZ > 0.0);

         double fT = Math.hypot(wrench.getLinearPartX(), wrench.getLinearPartY());
         assertTrue(fT / fZ < coefficientOfFriction);

         FramePoint2d cop = new FramePoint2d(planeFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, planeFrame);

         FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(contactState.getContactPoints2d());
         assertTrue(supportPolygon.isPointInside(cop));

      }
   }
}
