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
      joint1Position.setToZero(passiveJoint1.getFrameBeforeJoint());
      joint1Position.changeFrame(masterJoint.getFrameAfterJoint());
      masterL = Math.sqrt(Math.pow(joint1Position.getX(), 2) + Math.pow(joint1Position.getY(), 2) + Math.pow(joint1Position.getZ(), 2));
      
      joint2Position.setToZero(passiveJoint2.getFrameBeforeJoint());
      joint2Position.changeFrame(passiveJoint1.getFrameAfterJoint());
      L1 = Math.sqrt(Math.pow(joint2Position.getX(), 2) + Math.pow(joint2Position.getY(), 2) + Math.pow(joint2Position.getZ(), 2));
      
      joint3Position.setToZero(passiveJoint3.getFrameBeforeJoint());
      joint3Position.changeFrame(passiveJoint2.getFrameAfterJoint());
      L2 = Math.sqrt(Math.pow(joint3Position.getX(), 2) + Math.pow(joint3Position.getY(), 2) + Math.pow(joint3Position.getZ(), 2));
      
      masterJointPosition.setToZero(masterJoint.getFrameBeforeJoint());
      masterJointPosition.changeFrame(passiveJoint3.getFrameAfterJoint());
      L3 = Math.sqrt(Math.pow(masterJointPosition.getX(), 2) + Math.pow(masterJointPosition.getY(), 2) + Math.pow(masterJointPosition.getZ(), 2));
           
      // Close the loop
      fourBarCalculator = new FourBarCalculatorFromFastRunner(masterL, L1, L2, L3);
      fourBarCalculator.solveForAngleDAB(masterJointQ.getDoubleValue());
   }

}
