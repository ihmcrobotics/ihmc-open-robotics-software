package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public abstract class MomentumBasedController implements RobotController
{
   private static final long serialVersionUID = 1317956310066671754L;
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);
   protected final MomentumSolver solver;

   public MomentumBasedController(SixDoFJoint rootJoint, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
                                   LinearSolver<DenseMatrix64F> jacobianSolver, double controlDT)
   {
      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);
   }

   public void initialize()
   {
      solver.initialize();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }
}
