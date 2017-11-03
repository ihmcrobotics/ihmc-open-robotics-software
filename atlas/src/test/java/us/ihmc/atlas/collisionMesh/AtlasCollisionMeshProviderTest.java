package us.ihmc.atlas.collisionMesh;

import java.awt.Color;
import java.util.ArrayList;

import org.junit.Test;

import gnu.trove.map.hash.THashMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.avatar.collisionAvoidance.RobotCollisionMeshProvider;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public class AtlasCollisionMeshProviderTest
{
   @Test
   public void testAtlasCollisionMesh()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      RobotDescription atlasRobotDescription = atlasRobotModel.getRobotDescription();
      FullHumanoidRobotModel atlasFullRobotModel = atlasRobotModel.createFullRobotModel();
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(8);
      THashMap<RigidBody, FrameConvexPolytope> atlasCollisionMesh = meshProvider.createCollisionMeshesFromRobotDescription(atlasFullRobotModel, atlasRobotDescription);
      HumanoidFloatingRootJointRobot atlasFloatingRobotModel = atlasRobotModel.createHumanoidFloatingRootJointRobot(false, true);
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(atlasCollisionMesh.size(), true, atlasFloatingRobotModel);
      for(RigidBody rigidBody : ScrewTools.computeRigidBodiesAfterThisJoint(atlasFullRobotModel.getRootJoint()))
      {
         if(atlasCollisionMesh.get(rigidBody) != null)
            viz.addPolytope(atlasCollisionMesh.get(rigidBody));
         else
            PrintTools.debug("Getting a null for rigid body " + rigidBody.getName());
      }
      viz.update();
   }
   
   @Test
   public void testAtlasHeadCollisionMeshLoading()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      RobotDescription atlasRobotDescription = atlasRobotModel.getRobotDescription();
      FullHumanoidRobotModel atlasFullRobotModel = atlasRobotModel.createFullRobotModel();
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(8);
      RigidBody head = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(atlasFullRobotModel.getRootJoint()), "head")[0];
      LinkDescription headDescription = atlasRobotDescription.getLinkDescription("neck_ry");
      ArrayList<Point3D> pointList = meshProvider.getCollisionMeshPoints(headDescription.getCollisionMeshes(), new Vector3D());
      FrameConvexPolytope polytope = new FrameConvexPolytope(head.getBodyFixedFrame());
      FrameConvexPolytope polytope2 = new FrameConvexPolytope(head.getBodyFixedFrame());
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(2, true);
      viz.addPolytope(polytope, Color.BLUE);
      viz.addPolytope(polytope2, Color.RED);
      for(int i = 0; i < pointList.size() / 2; i++)
      {
         polytope.addVertex(pointList.get(i), Epsilons.ONE_TEN_THOUSANDTH);
         viz.updateNonBlocking();
      }
      for(int i = pointList.size() / 2; i < pointList.size(); i++)
      {
         polytope2.addVertex(pointList.get(i), Epsilons.ONE_TEN_THOUSANDTH);
         viz.updateNonBlocking();
      }
      viz.update();
      
   }
   
}
