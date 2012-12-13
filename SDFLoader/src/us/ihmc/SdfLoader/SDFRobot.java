package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;

public class SDFRobot extends Robot
{
   private static final long serialVersionUID = 5864358637898048080L;

   private final String resourceDirectory;
   private final HashMap<String, PinJoint> robotJoints = new HashMap<String, PinJoint>();

   private final FloatingJoint rootJoint;

   private final SideDependentList<ArrayList<GroundContactPoint>> groundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   
   public SDFRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap)
   {
      super(generalizedSDFRobotModel.getName());
      this.resourceDirectory = generalizedSDFRobotModel.getResourceDirectory();

      System.out.println("Creating root joints for root links");

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();

      if (rootLinks.size() > 1)
      {
         throw new RuntimeException("Can only accomodate one root link for now");
      }

      SDFLinkHolder rootLink = rootLinks.get(0);

      Vector3d offset = new Vector3d();
      Matrix3d rotation = new Matrix3d();
      rootLink.getTransformFromModelReferenceFrame().get(rotation, offset);
      rootJoint = new FloatingJoint(rootLink.getName(), generalizedSDFRobotModel.getRootOffset(), this);
      Link scsRootLink = createLink(rootLink, rotation);
      rootJoint.setLink(scsRootLink);
      addRootJoint(rootJoint);

      for (SDFJointHolder child : rootLink.getChilderen())
      {
         addJointsRecursively(child, rootJoint, MatrixTools.IDENTITY);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<GroundContactPoint> groundContactPointsForSide = new ArrayList<GroundContactPoint>();

         int i = 0;
         for (Vector3d groundContactPointOffset : sdfJointNameMap.getGroundContactPointOffset(robotSide))
         {
            String jointName = sdfJointNameMap.getJointBeforeFootName(robotSide);
            GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + jointName + "_" + i, groundContactPointOffset, this);
            robotJoints.get(jointName).addGroundContactPoint(groundContactPoint);
            groundContactPointsForSide.add(groundContactPoint);
            i++;

         }
         groundContactPoints.put(robotSide, groundContactPointsForSide);
      }
      
      
      Point3d centerOfMass = new Point3d();
      double totalMass = computeCenterOfMass(centerOfMass);
      System.out.println("Total mass: " + totalMass);

   }

   public void getRootJointToWorldTransform(Transform3D transform)
   {
      rootJoint.getTransformToWorld(transform);
   }

   public void setPositionInWorld(Vector3d offset)
   {
      rootJoint.setPosition(offset);
   }

   public PinJoint getJoint(String name)
   {
      return robotJoints.get(name);
   }

   private void addJointsRecursively(SDFJointHolder joint, Joint scsParentJoint, Matrix3d chainRotationIn)
   {
//      System.out.println("Adding joint " + joint.getName() + " to " + scsParentJoint.getName());
      Vector3d offset = new Vector3d();
      Matrix3d rotation = new Matrix3d();
      Transform3D transform = new Transform3D();

      joint.getTransformFromParentJoint().get(rotation, offset);
      transform.setRotation(chainRotationIn);
      transform.transform(offset);

      Matrix3d chainRotation = new Matrix3d(chainRotationIn);
      chainRotation.mul(rotation);

      Vector3d jointAxis = new Vector3d(joint.getAxis());
      PinJoint scsJoint = new PinJoint(SDFConversionsHelper.sanitizeJointName(joint.getName()), offset, this, jointAxis);
      scsJoint.setLink(createLink(joint.getChild(), chainRotation));
      scsParentJoint.addJoint(scsJoint);

      

      
//      scsJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), joint.getContactKd());
      scsJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 25.0);
      
      robotJoints.put(joint.getName(), scsJoint);

      for (SDFJointHolder child : joint.getChild().getChilderen())
      {
         addJointsRecursively(child, scsJoint, chainRotation);
      }

   }

   public boolean hasGroundContact(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPoints.get(robotSide))
      {
         if (groundContactPoint.isInContact())
         {
            return true;
         }
      }

      return false;
   }

   private Link createLink(SDFLinkHolder link, Matrix3d rotation)
   {
      SDFGraphics3DObject linkGraphics = new SDFGraphics3DObject(rotation, link.getVisuals(), resourceDirectory);

      Link scsLink = new Link(link.getName());
      scsLink.setLinkGraphics(linkGraphics);
      scsLink.setComOffset(link.getCoMOffset());
      scsLink.setMass(link.getMass());
      scsLink.setMomentOfInertia(link.getInertia());
      return scsLink;

   }

   public FrameVector getRootJointVelocity()
   {
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      rootJoint.getVelocity(ret.getVector());
      return ret;
   }
   
   public FrameVector getPelvisAngularVelocityInPelvisFrame(ReferenceFrame pelvisFrame)
   {
      Vector3d angularVelocity = rootJoint.getAngularVelocityInBody();
      return new FrameVector(pelvisFrame, angularVelocity);
   }



}
