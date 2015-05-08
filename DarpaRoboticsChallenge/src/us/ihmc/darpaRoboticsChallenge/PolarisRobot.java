package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

/**
 * This is a non-contactable Polaris model only for visualizing in SCS
 */

public class PolarisRobot extends Robot
{
   private final static String polarisModelFile = "models/polarisModel.obj";
   private final static String checkerBoardModelFile = "models/GFE/ihmc/calibration_cube.dae";
   
   private final FloatingJoint floatingJoint;
   private final Link polarisLink;
   private final Graphics3DObject polarisLinkGraphics;
   
   /**
    * Transforms between wheel, car, and checkerboard
    * 
    * Inverses are included so callers won't call invert() on the object
    */
   private static final RigidBodyTransform wheelToCarTransform = new RigidBodyTransform(), carToWheelTransform = new RigidBodyTransform();   
   private static final double wheelToCarX = 0.39, wheelToCarY = 0.39, wheelToCarZ = 1.27, steeringWheelPitchInDegrees = -33.0;
   
   private static final Vector3d checkerBoardToWheelTranslation = new Vector3d(0.7418, -0.798, -0.0642);
   private static final Matrix3d checkerBoardToWheelRotation = new Matrix3d(-0.8592, -0.0079, 0.5115, 0.0046, -1.0000, -0.0078, 0.5116, -0.0044, 0.8592);
   private static final RigidBodyTransform wheelToCheckerBoard = new RigidBodyTransform(), checkerBoardToWheel = new RigidBodyTransform(checkerBoardToWheelRotation, checkerBoardToWheelTranslation);

   public PolarisRobot(String name, RigidBodyTransform rootJointTransform)
   {
      super(name);

      polarisLink = new Link(name + "Link");
      polarisLinkGraphics = new Graphics3DObject();
      polarisLinkGraphics.addModelFile(polarisModelFile);
      polarisLink.setLinkGraphics(polarisLinkGraphics);

      polarisLink.setMass(1.0);
      polarisLink.setComOffset(new Vector3d());
      Matrix3d inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(1.0, 1.0, 1.0, Axis.Z);
      polarisLink.setMomentOfInertia(inertia);

      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3d(), this);
      floatingJoint.setRotationAndTranslation(rootJointTransform);
      floatingJoint.setLink(polarisLink);
      floatingJoint.setDynamic(false);
      
      Link checkerBoardLink = new Link(name + "CheckerBoardLink");
      Graphics3DObject checkerBoardGraphics = new Graphics3DObject();
      // value of 2.68 corresponds to real board with grid size of 3.35cm
      checkerBoardGraphics.scale(new Vector3d(2.68, 2.68, 0.1));
      checkerBoardGraphics.addModelFile(checkerBoardModelFile);
      checkerBoardLink.setLinkGraphics(checkerBoardGraphics);
      
      FloatingJoint checkerBoardJoint = new FloatingJoint(name + "CheckerBoardJoint", new Vector3d(), this);
      checkerBoardJoint.setRotationAndTranslation(new RigidBodyTransform(new AxisAngle4d(new Vector3d(0.0, 1.0, 0.0), - Math.PI / 2.0), new Vector3d(1.1, -0.5, 1.3)));
      checkerBoardJoint.setLink(checkerBoardLink);
      checkerBoardJoint.setDynamic(false);
      floatingJoint.addJoint(checkerBoardJoint);
      
      this.addRootJoint(floatingJoint);
   }
   
   static
   {
      wheelToCarTransform.setTranslation(wheelToCarX, wheelToCarY, wheelToCarZ);
      Matrix3d rotation = new Matrix3d();
      rotation.rotY(Math.toRadians(steeringWheelPitchInDegrees));
      wheelToCarTransform.setRotation(rotation);
      
      carToWheelTransform.set(wheelToCarTransform);
      carToWheelTransform.invert();
      
      wheelToCheckerBoard.set(checkerBoardToWheel);
      wheelToCheckerBoard.invert();
   }
   
   public static RigidBodyTransform getCarToWheelTransform()
   {
      return wheelToCarTransform;
   }
   
   public static RigidBodyTransform getWheelToCarTransform()
   {
      return carToWheelTransform;
   }
   
   public static RigidBodyTransform getCheckerBoardToWheelTransform()
   {
      return checkerBoardToWheel;
   }
   
   public static RigidBodyTransform getWheelToCheckerBoardTransform()
   {
      return wheelToCheckerBoard;
   }
}
