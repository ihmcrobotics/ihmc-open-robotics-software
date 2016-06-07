package us.ihmc.commonWalkingControlModules.momentumBasedController.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControllerTestHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class VirtualModelControlOptimizationControlModuleTest
{
   private static final double highAngularWeight = 10.0;
   private static final double highLinearWeight = 10.0;
   private static final double lowAngularWeight = 0.1;
   private static final double lowLinearWeight = 0.1;

   private static final double rhoMin = 0.5;
   private static final double rhoWeight = 0.00001;

   private static final double gravity = -9.81;

   @TestPlanAnnotations.DeployableTestMethod
   @Test(timeout = 30000)
   public void testLinearOptimization()
   {
      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.RobotLegs robotLegs = testHelper.createRobotLeg(gravity);

      CommonHumanoidReferenceFrames referenceFrames = robotLegs.getReferenceFrames();
      robotLegs.createContactPoints();

      SideDependentList<ContactableFoot> contactableFeet = robotLegs.getFootContactableBodies();
      ArrayList<ContactableFoot> contactableBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactableBodies.add(contactableFeet.get(robotSide));

      FrameVector linearMomentumRate = new FrameVector(null, new Vector3d(20.0, 20.0, 50.0));
      MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
      momentumRateCommand.setLinearMomentumRateOfChange(linearMomentumRate);
      momentumRateCommand.setWeights(lowAngularWeight, lowAngularWeight, lowAngularWeight, highLinearWeight, highLinearWeight, highLinearWeight);

      ArrayList<PlaneContactStateCommand> contactStateCommands = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
         planeContactStateCommand.setContactingRigidBody(robotLegs.getFoot(robotSide));
         planeContactStateCommand.setCoefficientOfFriction(0.5);
         planeContactStateCommand.clearContactPoints();
         for (FramePoint2d contactPoint : contactableFeet.get(robotSide).getContactPoints2d())
         {
            contactPoint.changeFrame(robotLegs.getSoleFrame(robotSide));
            planeContactStateCommand.addPointInContact(contactPoint);
         }
         contactStateCommands.add(planeContactStateCommand);
      }

      VirtualModelControlSolution virtualModelControlSolution = createAndSubmitOptimizationModule(robotLegs, referenceFrames, contactableBodies, momentumRateCommand,
            contactStateCommands);
      SpatialForceVector momentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      double epsilon = 1;
      JUnitTools.assertTuple3dEquals(linearMomentumRate.getVector(), momentumRateSolution.getLinearPart(), epsilon);
   }

   private VirtualModelControlSolution createAndSubmitOptimizationModule(FullRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames,
         ArrayList<ContactableFoot> contactableBodies, MomentumRateCommand momentumRateCommand, ArrayList<PlaneContactStateCommand> contactStateCommands)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      double controlDT = 0.005;

      DenseMatrix64F weight = momentumRateCommand.getWeightVector();
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings();
      double x = weight.get(3, 0);
      double y = weight.get(4, 0);
      double z = weight.get(5, 0);
      double pitch = weight.get(0, 0);
      double roll = weight.get(1, 0);
      double yaw = weight.get(2, 0);
      momentumOptimizationSettings.setMomentumWeight(x, y, z, pitch, yaw);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(rhoWeight);
      momentumOptimizationSettings.setRhoMin(rhoMin);

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(null, null, fullRobotModel.getOneDoFJoints(), momentumOptimizationSettings,
            referenceFrames, controlDT, -gravity, new GeometricJacobianHolder(), twistCalculator, contactableBodies, graphicsListRegistry, registry);

      VirtualModelControlOptimizationControlModule optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, fullRobotModel.getRootJoint(), registry);
      optimizationControlModule.initialize();
      optimizationControlModule.submitMomentumRateCommand(momentumRateCommand);
      for (int i = 0; i < contactStateCommands.size(); i++)
         optimizationControlModule.submitPlaneContactStateCommand(contactStateCommands.get(i));

      geometricJacobianHolder.compute();

      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      try
      {
         optimizationControlModule.compute(virtualModelControlSolution);
      }
      catch (VirtualModelControlModuleException e)
      {
         virtualModelControlSolution = e.getVirtualModelControlSolution();
         PrintTools.error("No solution is found");
      }

      return virtualModelControlSolution;
   }
}
