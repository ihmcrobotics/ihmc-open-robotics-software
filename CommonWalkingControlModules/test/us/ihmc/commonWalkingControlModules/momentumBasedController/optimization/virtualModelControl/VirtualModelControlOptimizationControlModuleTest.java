package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControllerTestHelper.RobotLeg;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControllerTestHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class VirtualModelControlOptimizationControlModuleTest
{
   private static final double linearMomentumX = 2.0;
   private static final double linearMomentumY = 0.0;
   private static final double linearMomentumZ = 7.5;

   private static final double angularMomentumYaw = 0.0;
   private static final double angularMomentumPitchRoll = 1.0;

   private static final double rhoWeight = 0.00001;

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testOptimization()
   {
      double gravity = -9.81;
      double controlDT = 0.005;
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      ReferenceFrame soleFrame = robotLeg.getSoleFrame();

      RigidBody[] endEffectors = {endEffector};

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(base, endEffector);

      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings();
      momentumOptimizationSettings.setMomentumWeight(linearMomentumX, linearMomentumY, linearMomentumZ, angularMomentumPitchRoll, angularMomentumYaw);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(rhoWeight);

      List<Point2d> contactPointsInSoleFrame = new ArrayList<Point2d>();
      contactPointsInSoleFrame.add(new Point2d(-VirtualModelControllerTestHelper.footLength / 2.0, -VirtualModelControllerTestHelper.footWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2d(-VirtualModelControllerTestHelper.footLength / 2.0, VirtualModelControllerTestHelper.footWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2d(VirtualModelControllerTestHelper.footLength / 2.0, -VirtualModelControllerTestHelper.toeWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2d(VirtualModelControllerTestHelper.footLength / 2.0, VirtualModelControllerTestHelper.toeWidth / 2.0));

      Point2d toeOffContactPoint = new Point2d(VirtualModelControllerTestHelper.footLength / 2.0, 0.0);

      ArrayList<ContactableFoot> contactableBodies = new ArrayList<>();
      ListOfPointsContactableFoot contactableFoot = new ListOfPointsContactableFoot(endEffector, soleFrame, contactPointsInSoleFrame, toeOffContactPoint);
      contactableBodies.add(contactableFoot);

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(null, endEffectors, controlledJoints, momentumOptimizationSettings, null, controlDT,
            gravity, new GeometricJacobianHolder(), null, contactableBodies, graphicsListRegistry, registry);

      FrameVector linearMomentumRate = new FrameVector(null, new Vector3d(10.0, 20.0, 50.0));
      MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
      momentumRateCommand.setLinearMomentumRateOfChange(linearMomentumRate);
      momentumRateCommand.setWeights(angularMomentumPitchRoll, angularMomentumPitchRoll, angularMomentumYaw, linearMomentumX, linearMomentumY, linearMomentumZ);

      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      for (FramePoint2d contactPoint : contactableFoot.getContactPoints2d())
         planeContactStateCommand.addPointInContact(contactPoint);

      VirtualModelControlOptimizationControlModule optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, registry);
      optimizationControlModule.initialize();
      optimizationControlModule.submitMomentumRateCommand(momentumRateCommand);
      optimizationControlModule.submitPlaneContactStateCommand(planeContactStateCommand);

      try
      {
         VirtualModelControlSolution virtualModelControlSolution = optimizationControlModule.compute();
      }
      catch (VirtualModelControlModuleException e)
      {
         PrintTools.error("No solution is found");
      }
   }
}
