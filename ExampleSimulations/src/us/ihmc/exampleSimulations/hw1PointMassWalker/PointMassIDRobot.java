package us.ihmc.exampleSimulations.hw1PointMassWalker;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.PrismaticJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class PointMassIDRobot
{
   // Joints, Links and frames
   private final PointMassSCSRobot robot;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   private final ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final RigidBody elevator;
   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevatorFrame", worldFrame, new RigidBodyTransform());

   private final SixDoFJoint bodyJointID;
   private final SideDependentList<RevoluteJoint> hipJointsID = new SideDependentList<>();
   private final SideDependentList<PrismaticJoint> kneeJointsID = new SideDependentList<>();
   
   private RigidBody bodyRigidBody;
   private final SideDependentList<RigidBody> upperRigidBody = new SideDependentList<>();
   private final SideDependentList<RigidBody> lowerRigidBody = new SideDependentList<>();
   
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ZUpFrame> soleZUpFrames = new SideDependentList<>();
   private final SideDependentList<YoGraphicReferenceFrame> soleFramesViz = new SideDependentList<>();
   private final MidFrameZUpFrame midSoleZUpFrame;

   // Jacobians
   private final SideDependentList<GeometricJacobian> legJacobians = new SideDependentList<>();
   
   // CoM, CoP and ICP calculations and plots
   private final TwistCalculator twistCalculator;
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
      
   private final double gravity;
   private final double totalRobotMass;
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   
   private final YoFramePoint yoCoM = new YoFramePoint("centerOfMass", worldFrame, registry);
   private final YoFrameVector yoCoMVelocity = new YoFrameVector("centerOfMassVelocity", worldFrame, registry);
   private final YoFramePoint2d yoICP = new YoFramePoint2d("capturePoint", worldFrame, registry);
   private final YoFramePoint yoCoP = new YoFramePoint("centerOfPressure", worldFrame, registry);
   private final SideDependentList<YoFramePoint> yoFootPositions = new SideDependentList<>();
   private final YoFrameVector measuredGroundReactionForce = new YoFrameVector("measuredGroundReactionForce", worldFrame, registry);

   // To update the ID Robot
   private double qd_x, qd_z, qd_wy;
   private double totalFz;
   
   private final RigidBodyTransform transformRootToWorld = new RigidBodyTransform();
   private final FrameVector bodyJointLinearVelocity = new FrameVector();
   private final FrameVector bodyJointAngularVelocity = new FrameVector();
   private final Twist bodyJointTwist = new Twist();

   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();  
   private final FramePoint2d centerOfMass2d = new FramePoint2d();
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d();
   private final FramePoint2d capturePoint = new FramePoint2d();

   private final FramePoint footPosition = new FramePoint();
   
   
   /**
    * Joints and Visualizers
    */
   public PointMassIDRobot(PointMassSCSRobot robot, YoVariableRegistry parentRegistry)
   {
      // Elevator 
      this.robot = robot;  //robot refers to the SCS robot
      elevator = new RigidBody("elevator", elevatorFrame);
      
      // Body
      bodyJointID = new SixDoFJoint("bodyJoint", elevator, elevatorFrame);
      bodyRigidBody = createRigidBodyFromLink(robot.getBody(), bodyJointID);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         // Hips
         PinJoint scsHipJoint = robot.getHipJoint(robotSide);
         String hipJointName = scsHipJoint.getName();
         Vector3d hipJointOffset = new Vector3d();
         Vector3d hipJointAxis = new Vector3d();
         scsHipJoint.getOffset(hipJointOffset);
         scsHipJoint.getJointAxis(hipJointAxis);        
         
         RevoluteJoint hipJoint = ScrewTools.addRevoluteJoint(hipJointName, bodyRigidBody, hipJointOffset, hipJointAxis);
         RigidBody upperRB = createRigidBodyFromLink(robot.getUpperLink(robotSide), hipJoint);
         hipJointsID.put(robotSide, hipJoint);
         upperRigidBody.put(robotSide, upperRB);
         
         // Knees
         SliderJoint scsKneeJoint = robot.getKneeJoint(robotSide);
         String kneeJointName = scsKneeJoint.getName();
         Vector3d kneeJointOffset = new Vector3d();
         Vector3d kneeJointAxis = new Vector3d();
         scsKneeJoint.getOffset(kneeJointOffset);
         scsKneeJoint.getJointAxis(kneeJointAxis);
         
         PrismaticJoint kneeJoint = ScrewTools.addPrismaticJoint(kneeJointName, upperRB, kneeJointOffset, kneeJointAxis);
         RigidBody lowerRB = createRigidBodyFromLink(robot.getLowerLink(robotSide), kneeJoint);
         kneeJointsID.put(robotSide, kneeJoint);
         lowerRigidBody.put(robotSide, lowerRB);
         
         // Jacobian
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.setTranslation(0.0, 0.0, - robot.getLowerLinkLength());
         ReferenceFrame soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(sidePrefix + "SoleFrame", kneeJoint.getFrameAfterJoint(), transformToParent);
         soleFrames.put(robotSide, soleFrame);
         
         ZUpFrame soleZUpFrame = new ZUpFrame(worldFrame, soleFrame, sidePrefix + "SoleZUpFrame");
         soleZUpFrames.put(robotSide, soleZUpFrame);
         
         GeometricJacobian jacobian = new GeometricJacobian(bodyRigidBody, lowerRB, soleFrame);
         legJacobians.put(robotSide, jacobian);
//         System.out.println(legJacobians);
      }
      
      for  (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         // Visualize feet location on plotter panel
         YoFramePoint yoFootPosition = new YoFramePoint(sidePrefix + "FootPosition", worldFrame, registry);
         yoFootPositions.put(robotSide, yoFootPosition);
         YoArtifactPosition footPositionArtifact = new YoArtifactPosition(sidePrefix + " Foot", yoFootPosition.getYoX(), yoFootPosition.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Green().getColor().get(), 0.009);
         artifactList.add(footPositionArtifact);
         
         // Visualize sole frames on SCS
         soleFramesViz.put(robotSide, new YoGraphicReferenceFrame(soleFrames.get(robotSide), registry, 0.15));
         yoGraphicsList.add(soleFramesViz.get(robotSide));
      }
      
      // Visualize body frame in SCS
      bodyFrameViz = new YoGraphicReferenceFrame(bodyJointID.getFrameAfterJoint(), registry, 0.3);
      yoGraphicsList.add(bodyFrameViz);
      
      // Visualize CoM, CP and CoP on plotter panel
      YoArtifactPosition centerOfMassArtifact = new YoArtifactPosition("Center of Mass", yoCoM.getYoX(), yoCoM.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Black().getColor().get(), 0.009);
      YoArtifactPosition capturePointArtifact = new YoArtifactPosition("Capture Point", yoICP.getYoX(), yoICP.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Blue().getColor().get(), 0.009);
      YoArtifactPosition centerOfPressureArtifact = new YoArtifactPosition("Center of Pressure", yoCoP.getYoX(), yoCoP.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Red().getColor().get(), 0.009);
      artifactList.add(centerOfMassArtifact);
      artifactList.add(capturePointArtifact);
      artifactList.add(centerOfPressureArtifact);
      
      // Initialize parameters for CoM, CoP and ICP calculations
      midSoleZUpFrame = new MidFrameZUpFrame("midSoleZUpFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      twistCalculator = new TwistCalculator(worldFrame, elevator);
      centerOfMassFrame =  new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);

      gravity = Math.abs(robot.getGravityZ());
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(elevator);
      omega0.set(Math.sqrt(gravity / 0.97));
        
      parentRegistry.addChild(registry);
   }
      
   /**
    * Rigid Bodies
    */
   private RigidBody createRigidBodyFromLink(Link link, InverseDynamicsJoint parentJoint)
   {
      String name = link.getName();
      double mass = link.getMass();
      Matrix3d momentOfInertiaToPack = new Matrix3d();
      Vector3d centerOfMassOffsetToPack = new Vector3d();
      
      link.getMomentOfInertia(momentOfInertiaToPack);
      link.getComOffset(centerOfMassOffsetToPack);
      
      return ScrewTools.addRigidBody(name, parentJoint, momentOfInertiaToPack, mass, centerOfMassOffsetToPack);
   }
   
   /**
    * Update ID robot --> positions and velocities from SCS robot + CoM, CoP, ICP
    */
   public void updateIDRobot()
   {
      // Body
      robot.getBodyJoint().getTransformToWorld(transformRootToWorld);
      transformRootToWorld.normalize();
      bodyJointID.setPositionAndRotation(transformRootToWorld); 
      ReferenceFrame bodyFrame = bodyJointID.getFrameAfterJoint();
      
      qd_x = robot.getQd_x().getDoubleValue();
      qd_z = robot.getQd_z().getDoubleValue();
      qd_wy = robot.getQd_wy().getDoubleValue();
      
      bodyJointLinearVelocity.setIncludingFrame(worldFrame, qd_x, 0.0, qd_z); //qd_x and qd_z are in world frame and I want them in body frame
      bodyJointLinearVelocity.changeFrame(bodyFrame);   
      bodyJointAngularVelocity.setIncludingFrame(bodyFrame, 0.0, qd_wy, 0.0);
      bodyJointTwist.set(bodyFrame, elevatorFrame, bodyFrame, bodyJointLinearVelocity, bodyJointAngularVelocity); //TODO why is the base frame the elevator instead of worldFrame?
      bodyJointID.setJointTwist(bodyJointTwist);
      
      // Hip, knee, soleZUpFrames and jacobians
      for (RobotSide robotSide : RobotSide.values)
      {
         hipJointsID.get(robotSide).setQ(robot.getQ_hip().get(robotSide).getDoubleValue());
         hipJointsID.get(robotSide).setQd(robot.getQd_hip().get(robotSide).getDoubleValue());

         kneeJointsID.get(robotSide).setQ(robot.getQ_knee().get(robotSide).getDoubleValue());
         kneeJointsID.get(robotSide).setQd(robot.getQd_knee().get(robotSide).getDoubleValue());
      }
      
      //Update frames
      elevator.updateFramesRecursively(); //REALLY important line!!

      for (RobotSide robotSide : RobotSide.values)
      {
         soleZUpFrames.get(robotSide).update();
         legJacobians.get(robotSide).compute();
      }

      midSoleZUpFrame.update();
      twistCalculator.compute(); //cool tool, but not used here
      
      //CoM
      centerOfMassFrame.update();
      centerOfMassJacobian.compute();
      centerOfMass.setToZero(centerOfMassFrame); 
      centerOfMass.changeFrame(worldFrame); 
      centerOfMassJacobian.packCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(worldFrame); //to make sure that we are actually in world frame
      centerOfMass2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMass);
      centerOfMassVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMassVelocity); 
      
      yoCoM.set(centerOfMass);
      yoCoMVelocity.set(centerOfMassVelocity);
      
      //ICP
      CapturePointCalculator.computeCapturePoint(capturePoint, centerOfMass2d, centerOfMassVelocity2d, omega0.getDoubleValue());
      yoICP.set(capturePoint);
      
      //CoP
      measuredGroundReactionForce.setToZero();
      bodyFrameViz.update();
      for(RobotSide robotside : RobotSide.values)
      {
         footPosition.setToZero(soleFrames.get(robotside));
         yoFootPositions.get(robotside).setAndMatchFrame(footPosition);
         soleFramesViz.get(robotside).update();
         measuredGroundReactionForce.add(robot.getFootGCPoint(robotside).getYoForce());
      }
      
      totalFz = measuredGroundReactionForce.getZ();
      
      if (totalFz < 1.0e-3)
      {
         yoCoP.setToNaN();
      }
      else
      {
         double alpha = robot.getFootGCPoint(RobotSide.RIGHT).getYoForce().getZ() / totalFz; //fraction of the total Fz represented by the FzRightFoot
         yoCoP.interpolate(yoFootPositions.get(RobotSide.LEFT), yoFootPositions.get(RobotSide.RIGHT), alpha);
      } 
   }
   
   /**
    * Update SCS Robot ---> set torques from ID robot
    */
   public void updateSCSRobot()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.setHipTau(robotSide, hipJointsID.get(robotSide).getTau());
         robot.setKneeTau(robotSide, kneeJointsID.get(robotSide).getTau());
      }
   }
   
   /**
    * Getter, setters and others 
    */
   public SideDependentList<YoFramePoint> getYoFootPositions()
   {
      return yoFootPositions;
   }
   
   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }
   
   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   public MidFrameZUpFrame getMidSoleZUpFrame()
   {
      return midSoleZUpFrame;
   }
   
   public GeometricJacobian getLegJacobian(RobotSide robotSide)
   {
      return legJacobians.get(robotSide);
   }
   
   public void getCoM(FramePoint2d comToPack)   
   {
      yoCoM.getFrameTuple2dIncludingFrame(comToPack);
   }
   
   public void getCoM(FramePoint centerOfMassToPack)
   {
      yoCoM.getFrameTupleIncludingFrame(centerOfMassToPack);
   }

   public void getCapturePoint(FramePoint2d capturePoinToPack)
   {
      yoICP.getFrameTuple2dIncludingFrame(capturePoinToPack);
   }
   
   public double getTotalRobotMass()
   {
      return totalRobotMass;
   }
   
   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }
   
   public double getGravity()
   {
      return gravity;
   }
   
   public double getXVelocity() 
   {
      return robot.getQd_x().getDoubleValue();
   }
   
   public YoFramePoint2d getYoCapturePoint()
   {
      return yoICP;
   }
   
   // Joints
   public SixDoFJoint getRootJoint()
   {
      return bodyJointID;
   }
   
   public RevoluteJoint getHip(RobotSide robotSide)
   {
      return hipJointsID.get(robotSide);
   }
   
   public PrismaticJoint getKnee(RobotSide robotSide)
   {
      return kneeJointsID.get(robotSide);
   }
   
   // Rigid bodies
   public RigidBody getLowerRigidBody(RobotSide robotSide)
   {
      return lowerRigidBody.get(robotSide);
   }
   
   // Body
   public double getBodyPitch()
   {
      return robot.getBodyJoint().getQ_rot().getDoubleValue();
   }
   
   //Hip
   public double getHipPitch(RobotSide robotSide)
   {
      return robot.getHipJoint(robotSide).getQ().getDoubleValue();
   }
   
   // Feet
   public void getFootLinearVelocity(RobotSide robotSide, FrameVector linearVelocityToPack)
   {
      robot.getFootGCPoint(robotSide).getYoVelocity().getFrameTupleIncludingFrame(linearVelocityToPack);
   }
   
   private final Vector3d footForce = new Vector3d();
   private YoGraphicReferenceFrame bodyFrameViz;
   
   public double getFootForce(RobotSide robotSide)
   {
      GroundContactPoint footGroundContactPoint = robot.getFootGCPoint(robotSide);
      footGroundContactPoint.getForce(footForce);
      return footForce.length();
   }
}
