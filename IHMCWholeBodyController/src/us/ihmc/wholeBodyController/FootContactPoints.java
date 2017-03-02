package us.ihmc.wholeBodyController;

import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface FootContactPoints
{
   /**
    * Creates the contact points used by the simulation. Not necessarily limited to the feet. A map is returned that
    * can hold contact points for each rigid body. The key of the map is the name of the parent joint of that body.
    * The contact points are expected in body frame.
    *
    * @param footLength
    * @param footWidth
    * @param toeWidth
    * @param jointMap
    * @param soleToAnkleFrameTransforms
    * @return
    */
   public Map<String, List<Tuple3DBasics>> getSimulationContactPoints(double footLength, double footWidth, double toeWidth, DRCRobotJointMap jointMap,
         SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms);

   /**
    * Creates contact points to be used inside the controller. The points are 2d and expected in sole frame (aligned
    * with the foot plane). A set of contact points has to be specified for each side of the robot.
    *
    * @param footLength
    * @param footWidth
    * @param toeWidth
    * @return
    */
   public SideDependentList<List<Tuple2DBasics>> getControllerContactPoints(double footLength, double footWidth, double toeWidth);

   /**
    * Creates the toe off contact point used by the controller for each robot side. The point is in 2d and expected
    * in sole frame.
    *
    * @param footLength
    * @param footWidth
    * @param toeWidth
    * @return
    */
   public SideDependentList<Tuple2DBasics> getToeOffContactPoints(double footLength, double footWidth, double toeWidth);

   /**
    * Allows switching between different sets of ground parameters.
    *
    * @return
    */
   public boolean useSoftContactPointParameters();
}