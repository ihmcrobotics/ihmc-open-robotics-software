package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;

public class FourBarKinematicLoop
{
   /*
    *  Representation of the four bar with name correspondences:
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
   private double masterL, L1, L2, L3;
   
//   private final FourBarIDCalculator fourBarCalculator;

   private final FramePoint joint2Position;
   
 public FourBarKinematicLoop(String name, YoVariableRegistry registry, RevoluteJoint masterJoint, PassiveRevoluteJoint passiveJoint1, PassiveRevoluteJoint passiveJoint2, PassiveRevoluteJoint passiveJoint3) 
 {
    this.masterJoint = masterJoint;
    this.passiveJoint1 = passiveJoint1;
    this.passiveJoint2 = passiveJoint2;
    this.passiveJoint3 = passiveJoint3;
       
    masterJointQ = new DoubleYoVariable(name + "MasterJointQ", registry);
    masterJointQ.set(masterJoint.getQ());
    
   
    
    
    joint2Position = new FramePoint();
//    
//    // Link lengths
//    joint2Position.setToZero(passiveJoint1.getFrameBeforeJoint());
//    joint2Position.changeFrame(passiveJoint2.getFrameAfterJoint());
//    joint2Position.getPoint().
    
    
//    masterJoint.updateFramesRecursively();
    // getJointLimits  -- depending on these limits only one output angle solution will be valid
    // RevoluteJoint hipJoint = ScrewTools.addRevoluteJoint(hipJointName, bodyRigidBody, hipJointOffset, hipJointAxis);
    // RigidBody upperRB = createRigidBodyFromLink(robot.getUpperLink(robotSide), hipJoint);
    
//    fourBarCalculator = new FourBarIDCalculator(masterL, L1, L2, L3, true);
//    
//    double resultAngle = fourBarCalculator.calculateOutputAngleFromInputAngle(masterJointQ.getDoubleValue());
    
 }
 
 
}
