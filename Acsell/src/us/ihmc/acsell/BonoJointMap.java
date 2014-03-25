package us.ihmc.acsell;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.TransformTools;

/**
 * Created by dstephen on 2/14/14.
 */
public class BonoJointMap extends AxlJointMap
{
   public static final String chestName = "utorso";

   public static final double ankleHeight = 0.01;
   public static final double footForward = 0.202;
   public static final double footBack = 0.05;
   public static final double footWidth = 0.152;

   public static final double thighLength = 0.37694;
   public static final double shinLength = 0.42164;
   public static final double legLength = thighLength + shinLength;
   
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_ROLL, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_YAW };

   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<>();

   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<>();
   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();
   private final String[] forceSensorNames;
   
   private final SideDependentList<List<Pair<String, Vector3d>>> footGroundContactPoints = new SideDependentList<>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPointsForController = new SideDependentList<>();
   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<>();

   private final SideDependentList<Transform3D> ankleToSoleFrameTransforms = new SideDependentList<Transform3D>();

   private final BonoRobotModel robotModel;
   public BonoJointMap(BonoRobotModel robotModel)
   {
      super();
      this.robotModel = robotModel;
      for (RobotSide robotSide : RobotSide.values())
      {
         String robotSideLowerCaseFirstLetter = robotSide.getSideNameFirstLetter().toLowerCase();

         jointBeforeFeetNames.put(robotSide, robotSideLowerCaseFirstLetter + "_leg_lax");
         feetForceSensorNames.put(robotSide, jointBeforeFeetNames.get(robotSide));

         footGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         footGroundContactPointsForController.put(robotSide, new ArrayList<Point2d>());
         
         Transform3D ankleToSoleFrameTransform = TransformTools.yawPitchDegreesTransform(new Vector3d(0.0, 0.0, -ankleHeight), 0.0, 10.0);
         ankleToSoleFrameTransforms.put(robotSide, ankleToSoleFrameTransform);

         ArrayList<Vector3d> gcs = new ArrayList<>();
         
         gcs.add(new Vector3d(footForward, -footWidth / 2.0, -ankleHeight));
         gcs.add(new Vector3d(footForward, footWidth / 2.0, -ankleHeight));
         gcs.add(new Vector3d(-footBack, -footWidth / 2.0, -ankleHeight));
         gcs.add(new Vector3d(-footBack, footWidth / 2.0, -ankleHeight));
         
         for (int i = 0; i < gcs.size(); i++)
         {
            Vector3d gc = gcs.get(i);
            footGroundContactPointsForController.get(robotSide).add(new Point2d(gc.x, gc.y));
            ankleToSoleFrameTransform.transform(gc);
            footGroundContactPoints.get(robotSide).add(new Pair<>(getJointBeforeFootName(robotSide), gc));
         }
      }

      forceSensorNames = new String[]{feetForceSensorNames.get(RobotSide.LEFT), feetForceSensorNames.get(RobotSide.RIGHT)};
      
      spineJointNames.put("back_lbx", SpineJointName.SPINE_ROLL);
      spineJointNames.put("back_mby", SpineJointName.SPINE_PITCH);
      spineJointNames.put("back_ubz", SpineJointName.SPINE_YAW);

      for (String spineJoint : spineJointNames.keySet())
      {
         jointRoles.put(spineJoint, JointRole.SPINE);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         jointNameGroundContactPointMap.addAll(footGroundContactPoints.get(robotSide));
      }
      
      for (RobotSide robotSide : RobotSide.values())
      {
         ArrayList<Point2d> contactPointList = new ArrayList<Point2d>();
         
         for (Pair<String, Vector3d> contactPoint3d : footGroundContactPoints.get(robotSide))
         {
            double x = contactPoint3d.second().x;
            double y = contactPoint3d.second().y;
            contactPointList.add(new Point2d(x, y));
         }
         
         footGroundContactPointsForController.put(robotSide, contactPointList);
      }
   }

   @Override
   public String getModelName()
   {
      return "bono";
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return spineJointNames.get(jointName);
   }

   @Override
   public String getChestName()
   {
      return chestName;
   }

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return jointBeforeFeetNames.get(robotSide);
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   @Override
   public SideDependentList<Transform3D> getAnkleToSoleFrameTransform()
   {
      return ankleToSoleFrameTransforms;
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController()
   {
      return footGroundContactPointsForController;
   }

   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public DRCRobotModel getSelectedModel()
   {
      return robotModel;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public SideDependentList<String> getJointBeforeThighNames()
   {
      return BonoOrderedJointNames.getJointBeforeThighNames();
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return BonoOrderedJointNames.jointNames;
   }
}
