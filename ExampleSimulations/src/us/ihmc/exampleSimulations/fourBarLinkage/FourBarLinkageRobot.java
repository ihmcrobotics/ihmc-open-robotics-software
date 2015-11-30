package us.ihmc.exampleSimulations.fourBarLinkage;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class FourBarLinkageRobot extends Robot
{
   private static final boolean SHOW_CONTACT_GRAPHICS = true;
   private final ExternalForcePoint efpJoint0, efpJoint4;

   private final ArrayList<PinJoint> joints = new ArrayList<PinJoint>();

   private static final MethodToCloseLoop METHOD_TO_CLOSE_LOOP = MethodToCloseLoop.PD_CONTROLLER;

   public FourBarLinkageRobot(String name, FourBarLinkageParameters fourBarLinkageParameters, Vector3d offsetWorld, YoVariableRegistry registry)
   {
      super("fourBarLinkageRobot" + name);

      Vector3d offsetJoint01 = new Vector3d(0, 0, fourBarLinkageParameters.linkageLength_1);
      Vector3d offsetJoint12 = new Vector3d(0, 0, fourBarLinkageParameters.linkageLength_2);
      Vector3d offsetJoint23 = new Vector3d(0, 0, fourBarLinkageParameters.linkageLength_3);

      PinJoint pinJoint0 = new PinJoint(name + "PinJoint0", offsetWorld, this, Axis.Y);
      pinJoint0.setInitialState(fourBarLinkageParameters.initial_angle_0, 0.3);
      Link fourBarLink1 = createFourBarLink("fourBarLink1" + name, fourBarLinkageParameters.linkageLength_1, fourBarLinkageParameters.radius_1,
            fourBarLinkageParameters.mass_1);
      pinJoint0.setLink(fourBarLink1);
      pinJoint0.setDamping(0.0);
      this.addRootJoint(pinJoint0);

      PinJoint pinJoint1 = new PinJoint(name + "PinJoint1", offsetJoint01, this, Axis.Y);
      pinJoint1.setInitialState(fourBarLinkageParameters.initial_angle_1, 0.5);
      Link fourBarLink2 = createFourBarLink("fourBarLink2" + name, fourBarLinkageParameters.linkageLength_2, fourBarLinkageParameters.radius_2,
            fourBarLinkageParameters.mass_2);
      pinJoint1.setLink(fourBarLink2);
      pinJoint1.setDamping(fourBarLinkageParameters.damping_1);
      pinJoint0.addJoint(pinJoint1);

      PinJoint pinJoint2 = new PinJoint(name + "PinJoint2", offsetJoint12, this, Axis.Y);
      pinJoint2.setInitialState(fourBarLinkageParameters.initial_angle_2, -0.2);
      Link fourBarLink3 = createFourBarLink("fourBarLink3" + name, fourBarLinkageParameters.linkageLength_3, fourBarLinkageParameters.radius_3,
            fourBarLinkageParameters.mass_3);
      pinJoint2.setLink(fourBarLink3);
      pinJoint2.setDamping(fourBarLinkageParameters.damping_2);
      pinJoint1.addJoint(pinJoint2);

      PinJoint pinJoint3 = new PinJoint(name + "PinJoint3", offsetJoint23, this, Axis.Y);
      pinJoint3.setInitialState(0.5 * Math.PI, 0.7); // TODO: hack, determine this correctly
      Link fourBarLink4 = createFourBarLink("fourBarLink4" + name, fourBarLinkageParameters.linkageLength_4, fourBarLinkageParameters.radius_4,
            fourBarLinkageParameters.mass_4);
      pinJoint3.setLink(fourBarLink4);
      pinJoint3.setDamping(fourBarLinkageParameters.damping_3);
      pinJoint2.addJoint(pinJoint3);

      efpJoint4 = new ExternalForcePoint(name + "efp_4", new Vector3d(0.0, 0.0, fourBarLinkageParameters.linkageLength_4), this);
      efpJoint0 = new ExternalForcePoint(name + "efp_0", new Vector3d(), this);

      pinJoint3.addExternalForcePoint(efpJoint4);
      pinJoint0.addExternalForcePoint(efpJoint0);

      joints.add(pinJoint0);
      joints.add(pinJoint1);
      joints.add(pinJoint2);
      joints.add(pinJoint3);

      // Loop closure constraint
      switch (METHOD_TO_CLOSE_LOOP)
      {
      case PD_CONTROLLER:
         FourBarLinkageSimpleClosedLoopConstraintController fourBarLinkageSimpleClosedLoopConstraintController = new FourBarLinkageSimpleClosedLoopConstraintController(
               this);
         fourBarLinkageSimpleClosedLoopConstraintController.setClosureJointDamping(fourBarLinkageParameters.damping_0);
         setController(fourBarLinkageSimpleClosedLoopConstraintController);
         break;
      case SIMPLE_FUNCTION_TO_INTEGRATE:
         FourBarLinkagePDConstraintToIntegrate fourBarLinkagePDConstraintToIntegrate = new FourBarLinkagePDConstraintToIntegrate("fourBarIntegratedConstraint",
               efpJoint0, efpJoint4, getRobotsYoVariableRegistry());
         fourBarLinkagePDConstraintToIntegrate.setStiffness(1000.0);
         fourBarLinkagePDConstraintToIntegrate.setDamping(100.0);

         addFunctionToIntegrate(fourBarLinkagePDConstraintToIntegrate);
         break;
      case STIFF_AXIAL_FUNCTION_TO_INTEGRATE:
         FourBarLinkageConstraintToIntegrate fourBarLinkageConstraintToIntegrate = new FourBarLinkageConstraintToIntegrate("fourBarIntegratedConstraint",
               efpJoint0, efpJoint4, getRootJoints().get(0), getRobotsYoVariableRegistry());
         fourBarLinkageConstraintToIntegrate.setAxialStiffness(100.0);
         fourBarLinkageConstraintToIntegrate.setAxialDamping(10.0);
         fourBarLinkageConstraintToIntegrate.setRadialStiffness(1000.0);
         fourBarLinkageConstraintToIntegrate.setRadialDamping(100.0);

         addFunctionToIntegrate(fourBarLinkageConstraintToIntegrate);
         break;
      }

      // TODO add torque to efp to simulate damping

      if (SHOW_CONTACT_GRAPHICS)
      {
         double radius = 0.1;

         Graphics3DObject joint1Graphics = pinJoint0.getLink().getLinkGraphics();
         joint1Graphics.identity();
         joint1Graphics.translate(efpJoint0.getOffsetCopy());
         joint1Graphics.addSphere(radius, YoAppearance.LemonChiffon());

         Graphics3DObject joint4Graphics = pinJoint3.getLink().getLinkGraphics();
         joint4Graphics.identity();
         joint4Graphics.translate(efpJoint4.getOffsetCopy());
         joint4Graphics.addSphere(2 * radius, YoAppearance.Glass());

         //         Graphics3DObject joint4FixGraphics = pinJoint1.getLink().getLinkGraphics();
         //         joint4FixGraphics.identity();
         //         joint4FixGraphics.translate(efpJointFixJoint2InWorld.getOffsetCopy());
         //         joint4FixGraphics.addSphere(radius, YoAppearance.Yellow());
      }
   }

   private Link createFourBarLink(String linkName, double length, double radius, double mass)
   {
      Link link = new Link(linkName);
      Matrix3d inertiaCylinder = createInertiaMatrixCylinder(linkName, length, radius, mass);
      link.setMomentOfInertia(inertiaCylinder);
      link.setMass(mass);
      link.setComOffset(new Vector3d(0.0, 0.0, length / 2.0));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(length, radius, YoAppearance.HotPink());
      link.setLinkGraphics(linkGraphics);
      link.addCoordinateSystemToCOM(0.35);
      return link;
   }

   public ExternalForcePoint getEfpJoint0()
   {
      return efpJoint0;
   }

   public ExternalForcePoint getEfpJoint4()
   {
      return efpJoint4;
   }

   private Matrix3d createInertiaMatrixCylinder(String linkName, double length, double radius, double mass)
   {
      Matrix3d inertiaCylinder = new Matrix3d();
      inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);
      return inertiaCylinder;
   }

   private enum MethodToCloseLoop
   {
      PD_CONTROLLER, SIMPLE_FUNCTION_TO_INTEGRATE, STIFF_AXIAL_FUNCTION_TO_INTEGRATE
   }

   public PinJoint getJoint(int i)
   {
      return joints.get(i);
   }
}
