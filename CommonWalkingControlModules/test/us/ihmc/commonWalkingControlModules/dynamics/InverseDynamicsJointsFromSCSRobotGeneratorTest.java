package us.ihmc.commonWalkingControlModules.dynamics;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class InverseDynamicsJointsFromSCSRobotGeneratorTest
{

   @Test
   public void testSinglePinJoint() throws SimulationExceededMaximumTimeException
   {
      Robot robot = new Robot("Test");
      
      final PinJoint joint1 = new PinJoint("joint1", new Vector3d(), robot, Axis.Z);
      Link link1 = new Link("link1");
      link1.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
      link1.setComOffset(new Vector3d());
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(0.1, 1.0, YoAppearance.Red());
      linkGraphics.translate(0.5, 0.0, 0.1);
      linkGraphics.addCylinder(0.1, 0.05, YoAppearance.Black());
      link1.setLinkGraphics(linkGraphics);
      joint1.setLink(link1);
      joint1.setQd(1.0);
      
      robot.addRootJoint(joint1);
      
      final InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(robot);
      
      
      final RigidBody elevator = generator.getElevator();
      final ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
      final RevoluteJoint revoluteJoint1 = generator.getInverseDynamicsRevoluteJoint(joint1);
      
      RobotController controller = new RobotController()
      {
         private final Random random = new Random(1492L);
         
         private final YoVariableRegistry registry = new YoVariableRegistry("Controller");
         private final DoubleYoVariable bodyYawID = new DoubleYoVariable("bodyYawID", registry);
         private final DoubleYoVariable bodyYaw = new DoubleYoVariable("bodyYaw", registry);
         private final DoubleYoVariable inverseDynamicsTau = new DoubleYoVariable("inverseDynamicsTau", registry);
         private final DoubleYoVariable previousTau = new DoubleYoVariable("previousTau", registry);
         private final DoubleYoVariable tauError = new DoubleYoVariable("tauError", registry);
         
         private final TwistCalculator twistCalculator = new TwistCalculator(inertialFrame, elevator);
         private final InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, 0.0);
         
         public void initialize()
         {            
         }

         public YoVariableRegistry getYoVariableRegistry()
         {
            return registry;
         }

         public String getName()
         {
            return "Controller";
         }

         public String getDescription()
         {
            return getName();
         }

         public void doControl()
         {
            previousTau.set(joint1.getTau().getDoubleValue());

            generator.updateInverseDynamicsRobotModelFromRobot();
            
            twistCalculator.compute();
            inverseDynamicsCalculator.compute();
            
            ReferenceFrame spinningFrame = revoluteJoint1.getFrameAfterJoint();
            FrameOrientation orientation = new FrameOrientation(spinningFrame);
            orientation.changeFrame(ReferenceFrame.getWorldFrame());
            bodyYawID.set(orientation.getYawPitchRoll()[0]);
            
            bodyYaw.set(joint1.getQ().getDoubleValue());
            bodyYaw.set(bodyYaw.getDoubleValue() % (2.0*Math.PI));
            if (bodyYaw.getDoubleValue() > Math.PI) bodyYaw.sub(2.0*Math.PI);
            if (bodyYaw.getDoubleValue() < -Math.PI) bodyYaw.add(2.0*Math.PI);
            
            double tau = random.nextGaussian() * 10.0;
            joint1.setTau(tau);
            
            revoluteJoint1.setQddDesired(joint1.getQDD().getDoubleValue());
            inverseDynamicsTau.set(revoluteJoint1.getTau());
            
            tauError.set(Math.abs(previousTau.getDoubleValue() - inverseDynamicsTau.getDoubleValue()));
            
            if (tauError.getDoubleValue() > 1e-7) throw new RuntimeException("InverseDynamicsTau doesn't match!");
         }
      };
      
      robot.setController(controller);
           
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.001, 1);
      scs.startOnAThread();
      
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      blockingSimulationRunner.simulateAndBlock(2.0);
      
      
      ThreadTools.sleepForever();
      
      blockingSimulationRunner.destroySimulation();
   }

}
