package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Test functionality of CoPPointsInFoot
 * @author ApoorvS
 *
 */
public class CoPPointsInFootTest
{
   private final double maxXInSoleFrame = -0.075;
   private final double minXInSoleFrame = -0.05;
   private final double maxYInSoleFrame = +0.05;
   private final double minYInSoleFrame = -0.075;
   private final double xToAnkle = 0.0;
   private final double yToAnkle = -0.15;
   private final double zToAnkle = 0.5;
   private final List<Point2D> footVertexList = Arrays.asList(new Point2D(maxXInSoleFrame, maxYInSoleFrame), new Point2D(maxXInSoleFrame, minYInSoleFrame),
                                                              new Point2D(minXInSoleFrame, maxYInSoleFrame), new Point2D(minXInSoleFrame, minYInSoleFrame));
   private final YoVariableRegistry registry = new YoVariableRegistry("CoPPointsInFootTestRegistry");
   private final FootSpoof footSpoof = new FootSpoof("DummyFoot", xToAnkle, yToAnkle, zToAnkle, footVertexList, 0.5);
//   private final ReferenceFrame[] framesToRegister = {ReferenceFrame.getWorldFrame(), footSpoof.getSoleFrame()};
   private final ReferenceFrame[] framesToRegister = {ReferenceFrame.getWorldFrame()};
   private CoPPointsInFoot copPointsInFoot;

   @Before
   public void setup()
   {
      copPointsInFoot = new CoPPointsInFoot(0, framesToRegister, registry);
   }

   @After
   public void clean()
   {
      copPointsInFoot.reset();
      registry.clear();
   }

   @Test
   public void testAddCoPPointToList()
   {
      assert (copPointsInFoot.getCoPPointList().isEmpty());
      copPointsInFoot.addWayPoint(CoPPointName.BALL_COP);
      copPointsInFoot.addWayPoint(CoPPointName.HEEL_COP);
      assert (copPointsInFoot.getCoPPointList().size() == 2);
      assert (copPointsInFoot.getCoPPointList().get(0).checkCoPPointMatch(CoPPointName.BALL_COP));
      assert (copPointsInFoot.getCoPPointList().get(1).checkCoPPointMatch(CoPPointName.HEEL_COP));
      copPointsInFoot.reset();
      assert (copPointsInFoot.getCoPPointList().isEmpty());
   }

   @Test
   public void testAddandSetIncludingFrame()
   {
      FramePoint3D testLocation = new FramePoint3D(footSpoof.getSoleFrame(), 0.01, 0.0, 0.01);
      assert (copPointsInFoot.getCoPPointList().isEmpty());
      copPointsInFoot.addAndSetIncludingFrame(CoPPointName.BALL_COP, 0.0, testLocation);
      assert (copPointsInFoot.getCoPPointList().size() == 1);

      List<CoPPointName> pointsList = copPointsInFoot.getCoPPointList();
      for (int i = 0; i < pointsList.size(); i++)
      {
         assert (copPointsInFoot.get(i).getPosition().epsilonEquals(testLocation, Epsilons.ONE_BILLIONTH));
      }
   }   
}
