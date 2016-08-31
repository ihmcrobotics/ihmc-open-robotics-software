package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

public class JointDescription
{
   private final ArrayList<JointDescription> childrenJointDescriptions = new ArrayList<>();

   private JointDescription parentJoint;
   private Vector3d offsetFromParentJoint = new Vector3d();

   private LinkDescription link;

   public void setParentJoint(JointDescription parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public void setOffsetFromParentJoint(Vector3d offset)
   {
      this.offsetFromParentJoint.set(offset);
   }

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public void getOffsetFromParentJoint(Vector3d offsetToPack)
   {
      offsetToPack.set(offsetFromParentJoint);
   }

   public LinkDescription getLink()
   {
      return link;
   }

   public void setLink(LinkDescription link)
   {
      this.link = link;
   }

   public void addJoint(JointDescription childJointDescription)
   {
      childrenJointDescriptions.add(childJointDescription);
   }

   public void addJointWrenchSensor(JointWrenchSensorDescription jointWrenchSensorDescription)
   {
      // TODO Auto-generated method stub

   }

   public void addGroundContactPoint(GroundContactPointDescription groundContactPointDescription)
   {
      // TODO Auto-generated method stub

   }

   public void addCameraMount(CameraMountDescription robotCam)
   {
      // TODO Auto-generated method stub

   }

   public void addIMUMount(IMUMountDescription imuMount)
   {
      // TODO Auto-generated method stub

   }
}
