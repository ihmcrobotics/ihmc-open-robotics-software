package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public class InefficientPushrodTransmissionJacobianTest
{
   private final double TOLERANCE = 1e-7;
   private final boolean DEBUG = false;
   private final boolean visualizeAndKeepUp = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInefficientPushrodTransmissionJacobianForAnklesAtZero()
   {
      Robot robot = new Robot("testPushrodTransmission");

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      InefficientPushrodTransmissionJacobian inefficientPushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, registry, null);
      
      double[][] jacobian = new double[2][2];

      pitch.set(0.0);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, null);
      assertJacobianEquals(jacobian, -0.0366712094326246, -0.0366712094326246, -0.034118686505983736, 0.034118686505983736);

   }

	@Ignore
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInefficientPushrodTransmissionJacobianForWaistAtZero()
   {
      Robot robot = new Robot("testPushrodTransmission");

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      InefficientPushrodTransmissionJacobian inefficientPushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, registry, null);
      
      double[][] jacobian = new double[2][2];

      roll.set(0.0);
      pitch.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, roll, pitch, jacobian, null);
      assertJacobianEquals(jacobian, -0.04520035766057378, 0.04520035766057378, -0.06336787027660956, -0.06336787027660956); // Regression. Need to double check with Solid Works numbers.
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInefficientPushrodTransmissionForAnkles()
   {
      Robot robot = new ValkyrieAnkleRobot();

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);

      YoGraphicsListRegistry yoGraphicsListRegistry;
      if (visualizeAndKeepUp)
      {
         yoGraphicsListRegistry = new YoGraphicsListRegistry();
      }
      else
      {
         yoGraphicsListRegistry = null;
      }

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.ANKLE;
      InefficientPushrodTransmissionJacobian inefficientPushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, registry,
            yoGraphicsListRegistry);

      SimulationConstructionSet scs = null;
      if (visualizeAndKeepUp)
      {
         scs = new SimulationConstructionSet(robot);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }

      double[][] jacobian = new double[2][2];

      if (visualizeAndKeepUp)
      {
         pitch.set(0.0);
         roll.set(0.0);
         computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);

         double increment = 0.1;
         for (pitch.set(-Math.PI / 3.0); pitch.getDoubleValue() < Math.PI / 3.0; pitch.add(increment))
         {
            for (roll.set(-Math.PI / 3.0); roll.getDoubleValue() < Math.PI / 3.0; roll.add(increment))
            {
               computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
            }
         }
      }

      pitch.set(0.0);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.0366712094326246, -0.0366712094326246, -0.034118686505983736, 0.034118686505983736);

      pitch.set(0.2);
      roll.set(0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.035509962933305175, -0.036202355875729446, -0.03181607828356141, 0.03640538461195576);

      pitch.set(-0.2);
      roll.set(0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03695254741929382, -0.034740329545336665, -0.02988230913579041, 0.03440182578269918);

      pitch.set(0.2);
      roll.set(-0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.036202355875729446, -0.035509962933305175, -0.03640538461195576, 0.03181607828356141);

      pitch.set(0.35);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03406729338743576, -0.03406729338743576, -0.03341354644555879, 0.03341354644555879);

      pitch.set(-0.35);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03440991379530292, -0.03440991379530292, -0.030355910924449715, 0.030355910924449715);

      pitch.set(0.0);
      roll.set(0.25);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.037679153131957736, -0.03539813540952868, -0.02679783281436968, 0.038150540900731125);

      if (visualizeAndKeepUp)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
//	@Test(timeout = 30000)
   public void testInefficientPushrodTransmissionForWaist()
   {
      Robot robot = new ValkyrieWaistRobot();      

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);

      YoGraphicsListRegistry yoGraphicsListRegistry;
      if (visualizeAndKeepUp)
      {
         yoGraphicsListRegistry = new YoGraphicsListRegistry();
      }
      else
      {
         yoGraphicsListRegistry = null;
      }

      PushRodTransmissionJoint pushRodTransmissionJoint = PushRodTransmissionJoint.WAIST;
      InefficientPushrodTransmissionJacobian inefficientPushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, registry,
            yoGraphicsListRegistry);

      SimulationConstructionSet scs;
      if (visualizeAndKeepUp)
      {
         scs = new SimulationConstructionSet(robot);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }
      else
      {
         scs = null;
      }

      double[][] jacobian = new double[2][2];

      if (visualizeAndKeepUp)
      {
         pitch.set(0.0);
         roll.set(0.0);
         computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
         
         double increment = 0.1;
         for (roll.set(-Math.PI / 3.0); roll.getDoubleValue() < Math.PI / 3.0; roll.add(increment))
         {
         for (pitch.set(-Math.PI / 3.0); pitch.getDoubleValue() < Math.PI / 3.0; pitch.add(increment))
         {
            
               computeAndPrint(inefficientPushrodTransmissionJacobian, roll, pitch, jacobian, scs);
            }
         }
      }

      pitch.set(0.0);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.045200359335075935, 0.045200359335075935, -0.06336787278836283, -0.06336787278836283);

      pitch.set(0.2);
      roll.set(0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.04230625056188121, 0.04324814514161008, -0.06204870021273448, -0.06848251178654614);

      pitch.set(-0.2);
      roll.set(0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.04324814514161008, 0.04230625056188121, -0.06848251178654614, -0.06204870021273448);

      pitch.set(0.2);
      roll.set(-0.1);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.047953520032603066, 0.04505140974373653, -0.056019426591516215, -0.06263063790754238);

      pitch.set(0.35);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.044046573174072956, 0.042535787548547144, -0.05437474030401962, -0.0662728724367141);

      pitch.set(-0.35);
      roll.set(0.0);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.042535787548547144, 0.044046573174072956, -0.0662728724367141, -0.05437474030401962);

      pitch.set(0.0);
      roll.set(0.25);
      computeAndPrint(inefficientPushrodTransmissionJacobian, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.04124545702816225, 0.04124545702816225, -0.07040587092825361, -0.07040587092825361);

      if (visualizeAndKeepUp)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }

   }



   private void assertJacobianEquals(double[][] jacobian, double j00, double j01, double j10, double j11)
   {
      assertEquals(jacobian[0][0], j00, TOLERANCE);
      assertEquals(jacobian[0][1], j01, TOLERANCE);
      assertEquals(jacobian[1][0], j10, TOLERANCE);
      assertEquals(jacobian[1][1], j11, TOLERANCE);
      if(DEBUG){
         System.out.println("jacobian: "+ jacobian[1][1] + ", " + jacobian[1][0] +", "+ -jacobian[0][1] +", "+ -jacobian[0][0]);
         System.out.println("j: " + j00 + ", " + j01 + ", " + j10 + ", " + j11 );
      }

   }

   private void computeAndPrint(InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission, DoubleYoVariable topJointAngle,
                                DoubleYoVariable bottomJointAngle, double[][] jacobian, SimulationConstructionSet scs)
   {
      inefficientButReadablePushrodTransmission.computeJacobian(jacobian, topJointAngle.getDoubleValue(), bottomJointAngle.getDoubleValue());

      if (visualizeAndKeepUp)
      {
         if (scs != null) scs.tickAndUpdate();

         System.out.println("topJoint " + topJointAngle.getName() + " = " + topJointAngle.getDoubleValue());
         System.out.println("bottomJoint " + bottomJointAngle.getName() + " = " + bottomJointAngle.getDoubleValue());
         
         System.out.println("f5=1.0 -> tauTopJoint = " + jacobian[0][0] + ", tauBottomJoint = " + jacobian[1][0]);
         System.out.println("f6=1.0 -> tauTopJoint = " + jacobian[0][1] + ", tauBottomJoint = " + jacobian[1][1] + "\n");
      }
   }
   
   private class ValkyrieWaistRobot extends Robot
   {
      public ValkyrieWaistRobot()
      {
         super("ValkyrieWaistRobot");
         
         Graphics3DObject linkGraphics = new Graphics3DObject();

         double heightOfTopAxisAboveBottomAxis = 0.02032;

         linkGraphics.translate(0.0, 0.0, 1.0 - heightOfTopAxisAboveBottomAxis);
         linkGraphics.rotate(Math.PI/2.0, Axis.Z);
         linkGraphics.rotate(Math.PI/2.0, Axis.X);
         linkGraphics.translate(0.10705, 0.37547, -0.22417);

         linkGraphics.addModelFile("models/waistPushrods/waist_assem.STL", YoAppearance.Gold());

         this.addStaticLinkGraphics(linkGraphics);

         linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.1);
         this.addStaticLinkGraphics(linkGraphics);

      }
   }
   
   private class ValkyrieAnkleRobot extends Robot
   {
      public ValkyrieAnkleRobot()
      {
         super("ValkyrieAnkleRobot");
         
         Graphics3DObject linkGraphics = new Graphics3DObject();

//         double heightOfTopAxisAboveBottomAxis = 0.02032;

//         linkGraphics.translate(0.0, 0.0, 1.0 - heightOfTopAxisAboveBottomAxis);
         linkGraphics.translate(0.0, 0.0, 1.0);
         linkGraphics.rotate(Math.PI/2.0, Axis.X);
         linkGraphics.translate(-0.03635, -0.0864, -0.07257);

         linkGraphics.addModelFile("models/anklePushrods/ankle_assem.STL", YoAppearance.Gold());

         this.addStaticLinkGraphics(linkGraphics);

         linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.1);
         this.addStaticLinkGraphics(linkGraphics);

      }
   }
}
