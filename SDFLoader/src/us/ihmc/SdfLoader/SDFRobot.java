package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.MatrixTools;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;

public class SDFRobot extends Robot
{
   private static final long serialVersionUID = 5864358637898048080L;
   
   private final String resourceDirectory;
   
   private final ArrayList<Joint> rootJoints = new ArrayList<Joint>();
   
   private final HashMap<String, PinJoint> robotJoints = new HashMap<String, PinJoint>();

   public SDFRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      super(generalizedSDFRobotModel.getName());
      this.resourceDirectory = generalizedSDFRobotModel.getResourceDirectory();

      System.out.println("Creating root joints for root links");

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();
      
      
      for(SDFLinkHolder rootLink : rootLinks)
      {
         Vector3d offset = new Vector3d();
         Matrix3d rotation = new Matrix3d();
         rootLink.getTransformFromModelReferenceFrame().get(rotation, offset);
         Joint rootJoint = new PinJoint(rootLink.getName(), offset, this, Joint.X);
         Link scsRootLink = createLink(rootLink, rotation);
         rootJoint.setLink(scsRootLink);
         addRootJoint(rootJoint);
         rootJoints.add(rootJoint);
         
         for(SDFJointHolder child : rootLink.getChilderen())
         {
            addJointsRecursively(child, rootJoint, MatrixTools.IDENTITY);
         }
      }
      
   }
   
   public PinJoint getJoint(String name)
   {
      return robotJoints.get(name);
   }

   private void addJointsRecursively(SDFJointHolder joint, Joint scsParentJoint, Matrix3d chainRotationIn)
   {
      System.out.println("Adding joint " + joint.getName() + " to " + scsParentJoint.getName());
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
      
      robotJoints.put(joint.getName(), scsJoint);
      
      for(SDFJointHolder child : joint.getChild().getChilderen())
      {
         addJointsRecursively(child, scsJoint, chainRotation);
      }
      
   }
   
   private Link createLink(SDFLinkHolder link, Matrix3d rotation)
   {
      SDFLinkGraphics linkGraphics = new SDFLinkGraphics(rotation, link.getVisuals(), resourceDirectory);
      
      Link scsLink = new Link(link.getName());
      scsLink.setLinkGraphics(linkGraphics);
      scsLink.setComOffset(link.getCoMOffset());
      scsLink.setMass(link.getMass());
      scsLink.setMomentOfInertia(link.getInertia());
      return scsLink;

   }

}
