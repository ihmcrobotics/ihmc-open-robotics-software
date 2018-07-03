package us.ihmc.commonWalkingControlModules.contact;

import java.util.Comparator;
import java.util.List;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.lists.ArraySorter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class only enables the lowest N contact points when set in contact 
 * this is a bit of hack and could be the wrong approach when dealing with sloped surfaces
 */
public class YoLowestNPointsPlaneContactState extends YoPlaneContactState
{
   private final YoDouble numberOfContactsToActivate;
   private final YoContactPoint[] sortedContactPoints;
   private final ContactPointComparator contactPointComparator = new ContactPointComparator();
   private final TObjectIntHashMap<YoContactPoint> contactPointIndexLookup = new TObjectIntHashMap<>();

   public YoLowestNPointsPlaneContactState(String namePrefix, RigidBody rigidBody, ReferenceFrame planeFrame, List<FramePoint2D> contactFramePoints,
                                           double numberOfContactsToActivate, double coefficientOfFriction, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, rigidBody, planeFrame, contactFramePoints, coefficientOfFriction, parentRegistry);

      this.numberOfContactsToActivate = new YoDouble("numberOfContactsToActivate", registry);
      this.numberOfContactsToActivate.set(numberOfContactsToActivate);
      this.sortedContactPoints = new YoContactPoint[contactFramePoints.size()];
      
      List<YoContactPoint> contactPoints = getContactPoints();
      for(int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         sortedContactPoints[i] = yoContactPoint;
         contactPointIndexLookup.put(yoContactPoint, i);
      }
      sortContactPointsByZHeightInWorld();
   }

   @Override
   public void setFullyConstrained()
   {
      super.clear();
      
      sortContactPointsByZHeightInWorld();
      for(int i = 0; i < numberOfContactsToActivate.getDoubleValue(); i++)
      {
         YoContactPoint yoContactPoint = sortedContactPoints[i];
         setContactPointInContact(contactPointIndexLookup.get(yoContactPoint), true);
      }
      updateInContact();
   }
   
   private void sortContactPointsByZHeightInWorld()
   {
      ArraySorter.sort(sortedContactPoints, contactPointComparator);
   }

   private class ContactPointComparator implements Comparator<YoContactPoint>
   {
      private final FramePoint3D p1 = new FramePoint3D();
      private final FramePoint3D p2 = new FramePoint3D();
      
      /**
       * Returns -1 if cp1 z height is less than cp2 z height
       * Returns 0 if cp1 and cp2 have the same z height
       * Returns 1 if cp1 z height is greater than cp2 z height
       */
      @Override
      public int compare(YoContactPoint cp1, YoContactPoint cp2)
      {
         cp1.getPosition(p1);
         cp2.getPosition(p2);
         
         p1.changeFrame(ReferenceFrame.getWorldFrame());
         p2.changeFrame(ReferenceFrame.getWorldFrame());
         
         if(p1.getZ() < p2.getZ())
         {
            return -1;
         }
         
         if(p1.getZ() > p2.getZ())
         {
            return 1;
         }
         
         return 0;
      }

   }

}
