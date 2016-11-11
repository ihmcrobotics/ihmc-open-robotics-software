package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class QuadrupedFootContactableBodiesFactory
{
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<CommonQuadrupedReferenceFrames> referenceFrames = new RequiredFactoryField<>("referenceFrames");
   private final RequiredFactoryField<QuadrupedPhysicalProperties> physicalProperties = new RequiredFactoryField<>("physicalProperties");

   public QuadrantDependentList<ContactablePlaneBody> createFootContactableBodies()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      QuadrantDependentList<ContactablePlaneBody> footContactableBodies = new QuadrantDependentList<ContactablePlaneBody>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBody foot = fullRobotModel.get().getFoot(robotQuadrant);
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, referenceFrames.get().getFootFrame(robotQuadrant),
                                                                                                     physicalProperties.get()
                                                                                                                       .getFootGroundContactPoints(robotQuadrant));
         footContactableBodies.set(robotQuadrant, footContactableBody);
      }
      
      FactoryTools.disposeFactory(this);

      return footContactableBodies;
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setReferenceFrames(CommonQuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames.set(referenceFrames);
   }

   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties.set(physicalProperties);
   }
}
