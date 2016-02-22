package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorFromFastRunner;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences:
    *   
    *              masterL
    *      masterJ--------J1
    *            |\      /|
    *            | \    / |
    *          L3|  \  /  |L1
    *            |   \/   |
    *            |   /\   |
    *            |  /  \  |
    *            | /    \ |
    *            |/      \|
    *           J3--------J2
    *                L2
    */

   // private boolean prjIsPartOf4Bar = true;
   private final RevoluteJoint masterJoint;
   private final PassiveRevoluteJoint passiveJoint1, passiveJoint2, passiveJoint3;

   private final DoubleYoVariable masterJointQ;
   private final FramePoint masterJointPosition, joint1Position, joint2Position, joint3Position;
   private double masterL, L1, L2, L3;

   private final FourBarCalculatorFromFastRunner fourBarCalculator;

   public FourBarKinematicLoop(String name, YoVariableRegistry registry, RevoluteJoint masterJoint, PassiveRevoluteJoint passiveJoint1, PassiveRevoluteJoint passiveJoint2, PassiveRevoluteJoint passiveJoint3)
   {
      this.masterJoint = masterJoint;
      this.passiveJoint1 = passiveJoint1;
      this.passiveJoint2 = passiveJoint2;
      this.passiveJoint3 = passiveJoint3;

      joint1Position = new FramePoint();
      joint2Position = new FramePoint();
      joint3Position = new FramePoint();
      masterJointPosition = new FramePoint();
      
      masterJointQ = new DoubleYoVariable(name + "MasterJointQ", registry);
      masterJointQ.set(masterJoint.getQ());

      // Link lengths     
      masterL = getLinkLength(masterJoint, passiveJoint1, joint1Position);     
      L1 = getLinkLength(passiveJoint1, passiveJoint2, joint2Position);
      L2 = getLinkLength(passiveJoint2, passiveJoint3, joint3Position);
      L3 = getLinkLength(passiveJoint3, masterJoint, masterJointPosition);
           
      // Close the loop
      fourBarCalculator = new FourBarCalculatorFromFastRunner(masterL, L1, L2, L3);
      fourBarCalculator.solveForAngleDAB(masterJointQ.getDoubleValue());
   }
   
   public double getLinkLength(RevoluteJoint joint1, RevoluteJoint joint2, FramePoint positionJoint2) 
   {
      positionJoint2.setToZero(joint2.getFrameBeforeJoint());
      positionJoint2.changeFrame(joint1.getFrameAfterJoint());
      return Math.sqrt(Math.pow(positionJoint2.getX(), 2) + Math.pow(positionJoint2.getY(), 2) + Math.pow(positionJoint2.getZ(), 2));
   }
}
