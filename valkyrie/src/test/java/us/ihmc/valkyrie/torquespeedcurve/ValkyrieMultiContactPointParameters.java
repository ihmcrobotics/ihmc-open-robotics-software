package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ValkyrieMultiContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   private final List<String> planeContactNames = new ArrayList<>();
   private final Map<String, String> planeContactRigidBodyNames = new HashMap<>();
   private final Map<String, List<? extends Point2DReadOnly>> nameToContactPointsMap = new HashMap<>();
   private final Map<String, RigidBodyTransform> nameToFramePose = new HashMap<>();

   public ValkyrieMultiContactPointParameters(DRCRobotJointMap jointMap, ValkyriePhysicalProperties physicalProperties)
   {
      super(jointMap, physicalProperties.getFootWidth(), physicalProperties.getFootLength(), physicalProperties.getSoleToAnkleFrameTransforms());
   }

   public void addSingleContactPoint(String parentJointName, String bodyName, String contactName, Point3DReadOnly contactPointInParentJoint)
   {
      addSingleContactPoint(parentJointName, bodyName, contactName, new RigidBodyTransform(new Quaternion(), contactPointInParentJoint));
   }

   public void addSingleContactPoint(String parentJointName, String bodyName, String contactName, RigidBodyTransform contactPoseInParentJoint)
   {
      additionalContactRigidBodyNames.add(bodyName);
      additionalContactNames.add(contactName);
      additionalContactTransforms.add(contactPoseInParentJoint);
      addSimulationContactPoint(parentJointName, contactPoseInParentJoint.getTranslation());
   }

   public void addPlaneContact(String parentJointName, String bodyName, String contactName, RigidBodyTransform contactFramePose,
                               List<? extends Point2DReadOnly> contactPoints)
   {
      planeContactNames.add(contactName);
      planeContactRigidBodyNames.put(contactName, bodyName);
      nameToContactPointsMap.put(contactName, contactPoints);
      nameToFramePose.put(contactName, contactFramePose);

      for (Point2DReadOnly contactPoint : contactPoints)
      {
         Point3D contactPoint3D = new Point3D(contactPoint);
         contactFramePose.transform(contactPoint3D);
         addSimulationContactPoint(parentJointName, contactPoint3D);
      }
   }

   public List<String> getPlaneContactNames()
   {
      return planeContactNames;
   }

   public String getPlaneContactBodyName(String contactName)
   {
      return planeContactRigidBodyNames.get(contactName);
   }

   public List<? extends Point2DReadOnly> getPlaneContactPoints(String contactName)
   {
      return nameToContactPointsMap.get(contactName);
   }

   public RigidBodyTransform getPlaneContactFramePose(String contactName)
   {
      return nameToFramePose.get(contactName);
   }
}
