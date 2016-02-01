package us.ihmc.exampleSimulations.hw1PointMassWalker;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class PointMassSCSRobot extends Robot
{
   private final FloatingPlanarJoint bodyJoint;
   private final SideDependentList<PinJoint> hipJointsSCS = new SideDependentList<>();
   private final SideDependentList<SliderJoint> kneeJointsSCS = new SideDependentList<>();

   private final Link bodyLink;
   private final SideDependentList<Link> upperLinks = new SideDependentList<>();
   private final SideDependentList<Link> lowerLinks = new SideDependentList<>();
   
   private final SideDependentList<GroundContactPoint> footGCs = new SideDependentList<>();
   
   // Useful parameters
   private static final double upperLinkLength = 0.5;
   private static final double lowerLinkLength = 0.5;
   
   private final DoubleYoVariable q_x, q_z, q_pitch;
   private final DoubleYoVariable qd_x, qd_z, qd_wy;
   
   private final SideDependentList<DoubleYoVariable> qHip = new SideDependentList<>();
   private final SideDependentList<DoubleYoVariable> qdHip = new SideDependentList<>();
   private final SideDependentList<DoubleYoVariable> qKnee = new SideDependentList<>();
   private final SideDependentList<DoubleYoVariable> qdKnee = new SideDependentList<>();

   
   public PointMassSCSRobot()
   {
      super("PointMassRobot");
   
   /**
    * Joints
    */
      
      // Body
      bodyJoint = new FloatingPlanarJoint("bodyJoint", this);
      bodyLink = createBodyLink();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);
      
      // Legs and feet
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         PinJoint hipJoint = new PinJoint(sidePrefix + "Hip", new Vector3d(0.0, robotSide.negateIfRightSide(0.04), 0.0),this, Axis.Y);
         upperLinks.put(robotSide, createUpperLink(robotSide));
         hipJoint.setLink(upperLinks.get(robotSide));
         hipJointsSCS.put(robotSide, hipJoint);
         bodyJoint.addJoint(hipJoint);
         qHip.put(robotSide, hipJoint.getQ());
         qdHip.put(robotSide, hipJoint.getQD());
         
         SliderJoint kneeJoint = new SliderJoint(sidePrefix + "Knee", new Vector3d(0.0, 0.0, -upperLinkLength), this, Axis.Z);
         lowerLinks.put(robotSide, createLowerLink(robotSide));
         kneeJoint.setLink(lowerLinks.get(robotSide));
         kneeJoint.setLimitStops(0.0, lowerLinkLength, 10000.0, 5000.0);
         kneeJointsSCS.put(robotSide, kneeJoint);
         hipJoint.addJoint(kneeJoint);
         qKnee.put(robotSide, kneeJoint.getQ());
         qdKnee.put(robotSide, kneeJoint.getQD());
         
         GroundContactPoint groundContactPoint = new GroundContactPoint(sidePrefix + "GC", new Vector3d(0.0, 0.0, -lowerLinkLength), this);
         kneeJoint.addGroundContactPoint(groundContactPoint);
         footGCs.put(robotSide, groundContactPoint);
      }
      
      LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(this, yoVariableRegistry);
      linearGroundContactModel.setXYStiffness(1000.0);
      linearGroundContactModel.setXYDamping(150.0);
      linearGroundContactModel.setZStiffness(500.0);
      linearGroundContactModel.setZDamping(250.0);
      this.setGroundContactModel(linearGroundContactModel);
      
      // Some useful parameters 
      q_x = bodyJoint.getQ_t1();
      q_z = bodyJoint.getQ_t2();
      q_pitch = bodyJoint.getQ_rot();
      qd_x = bodyJoint.getQd_t1();
      qd_z = bodyJoint.getQd_t2();
      qd_wy = bodyJoint.getQd_rot();
      
      // Initial configuration
      initializeRobotConfiguration();
   }
  
   
   /**
    * Links
    */
   
   private Link createBodyLink()
   {
      Link bodyLink = new Link("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(10.0, 0.1, 0.1, 0.1);

      Graphics3DObject bodyGraphics = new Graphics3DObject();
      bodyGraphics.addSphere(0.175, YoAppearance.BlackMetalMaterial());
      bodyLink.setLinkGraphics(bodyGraphics);

      return bodyLink;
   }
   
   private Link createUpperLink(RobotSide robotSide)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      Link upperLink = new Link(sidePrefix + "upperLink");
      upperLink.setMassAndRadiiOfGyration(0.1, 0.1, 0.1, 0.1);
      upperLink.setComOffset(0.0, 0.0, -upperLinkLength / 2.0);

      Graphics3DObject upperLinkGraphics = new Graphics3DObject();
      upperLinkGraphics.translate(0.0, 0.0, -upperLinkLength);
      upperLinkGraphics.addCylinder(upperLinkLength, 0.05, YoAppearance.YellowGreen());
      upperLink.setLinkGraphics(upperLinkGraphics);

      return upperLink;
   }

   private Link createLowerLink(RobotSide robotSide)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      Link lowerLink = new Link(sidePrefix + "lowerLink");
      lowerLink.setMassAndRadiiOfGyration(0.1, 0.1, 0.1, 0.1);
      lowerLink.setComOffset(0.0, 0.0, -lowerLinkLength / 2.0);

      Graphics3DObject lowerLinkGraphics = new Graphics3DObject();
      lowerLinkGraphics.translate(0.0, 0.0, -lowerLinkLength);
      lowerLinkGraphics.addCylinder(lowerLinkLength, 0.045, YoAppearance.Yellow());
      lowerLink.setLinkGraphics(lowerLinkGraphics);

      return lowerLink;
   }
   
   /**
    * Initialize configuration
    */
   
   private void initializeRobotConfiguration()
   {
      q_z.set(0.97);
      qHip.get(RobotSide.LEFT).set(-0.2);
      qHip.get(RobotSide.RIGHT).set(0.2);
   }
   
   
   /**
    * Getters and setters
    */
   
   // Body vars
   public DoubleYoVariable getQ_x()
   {
      return q_x;
   }

   public DoubleYoVariable getQ_z()
   {
      return q_z;
   }

   public DoubleYoVariable getQ_pitch()
   {
      return q_pitch;
   }

   public DoubleYoVariable getQd_x()
   {
      return qd_x;
   }

   public DoubleYoVariable getQd_z()
   {
      return qd_z;
   }

   public DoubleYoVariable getQd_wy()
   {
      return qd_wy;
   }

   // Leg vars
   public SideDependentList<DoubleYoVariable> getQ_hip()
   {
      return qHip;
   }

   public SideDependentList<DoubleYoVariable> getQd_hip()
   {
      return qdHip;
   }

   public SideDependentList<DoubleYoVariable> getQ_knee()
   {
      return qKnee;
   }

   public SideDependentList<DoubleYoVariable> getQd_knee()
   {
      return qdKnee;
   }
   
   public double getLowerLinkLength()  //used to create the soleFrames
   {
      return lowerLinkLength;
   }
   
   // Links
   public Link getBody()
   {
      return bodyLink;
   }

   public Link getUpperLink(RobotSide robotSide)
   {
      return upperLinks.get(robotSide);
   }

   public Link getLowerLink(RobotSide robotSide)
   {
      return lowerLinks.get(robotSide);
   }
   
   // Joints
   public FloatingPlanarJoint getBodyJoint()
   {
      return bodyJoint;
   }

   public PinJoint getHipJoint(RobotSide robotSide)
   {
      return hipJointsSCS.get(robotSide);
   }

   public SliderJoint getKneeJoint(RobotSide robotSide)
   {
      return kneeJointsSCS.get(robotSide);
   }

   // GC points
   public GroundContactPoint getFootGCPoint(RobotSide robotSide)
   {
      return footGCs.get(robotSide);
   }
   
   // Set torques
   public void setHipTau(RobotSide robotSide, double tau)
   {
      hipJointsSCS.get(robotSide).setTau(tau);
   }

   public void setKneeTau(RobotSide robotSide, double tau)
   {
      kneeJointsSCS.get(robotSide).setTau(tau);
   }
}
