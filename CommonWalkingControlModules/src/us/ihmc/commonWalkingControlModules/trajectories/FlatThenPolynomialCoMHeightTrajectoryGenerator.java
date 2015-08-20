package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.OrbitalEnergyCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoUtilities.math.trajectories.YoPolynomial;


public class FlatThenPolynomialCoMHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final ReferenceFrame referenceFrame;
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   private final CommonHumanoidReferenceFrames referenceFrames;

   private static final int numberOfCoefficients = 6;

   private final double gravityZ;

   private final YoPolynomial heightSplineInFootFrame = new YoPolynomial("heightSplineInFootFrame", numberOfCoefficients, registry);
   private final DoubleYoVariable footX = new DoubleYoVariable("footX", registry);
   private final DoubleYoVariable footZ = new DoubleYoVariable("footZ", registry);

   private final DoubleYoVariable minXForSpline = new DoubleYoVariable("minXForSpline", registry);
   private final DoubleYoVariable maxXForSpline = new DoubleYoVariable("maxXForSpline", registry);

   private final DoubleYoVariable initialHeightAboveGround = new DoubleYoVariable("initialHeightAboveGround", registry);
   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

   private final DoubleYoVariable desiredComHeightInWorld = new DoubleYoVariable("desiredComHeightInWorld", registry);
   private final DoubleYoVariable desiredComHeightSlope = new DoubleYoVariable("desiredComHeightSlopeX", registry);
   private final DoubleYoVariable desiredComHeightSecondDerivative = new DoubleYoVariable("desiredComHeightSecondDerivativeX", registry);

   private final DoubleYoVariable orbitalEnergy = new DoubleYoVariable("orbitalEnergy", registry);

   private final YoPolynomial testHeightSplineInFootFrame = new YoPolynomial("testHeightSplineInFootFrame", numberOfCoefficients, registry);
   private final DoubleYoVariable deltaZ = new DoubleYoVariable("deltaZ", registry);

   public FlatThenPolynomialCoMHeightTrajectoryGenerator(double gravityZ, ReferenceFrame centerOfMassFrame, CenterOfMassJacobian centerOfMassJacobian,
         ReferenceFrame desiredHeadingFrame, SideDependentList<ContactablePlaneBody> bipedFeet, CommonHumanoidReferenceFrames referenceFrames,
         double nominalHeightAboveGround, double initialHeightAboveGround, YoVariableRegistry parentRegistry)
   {
      this.gravityZ = gravityZ;
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrame = desiredHeadingFrame;
      this.bipedFeet = bipedFeet;
      this.referenceFrames = referenceFrames;

      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
      this.initialHeightAboveGround.set(initialHeightAboveGround);
      parentRegistry.addChild(registry);
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep,
         List<PlaneContactState> contactStates)
   {
      if (supportLeg == null)
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            footZ.set(Math.min(findMinZOfGroundContactPoints(RobotSide.LEFT), findMinZOfGroundContactPoints(RobotSide.RIGHT)));
            heightSplineInFootFrame.setConstant(initialHeightAboveGround.getDoubleValue());
            hasBeenInitialized.set(true);
         }
      }
      else          
      {
         compute();
         footX.set(findMaxXOfGroundContactPoints(supportLeg));
         footZ.set(findMinZOfGroundContactPoints(supportLeg));

         double[] x = initializeSpline(transferToAndNextFootstepsData, supportLeg, heightSplineInFootFrame, footX.getDoubleValue(), footZ.getDoubleValue());
         minXForSpline.set(x[0]);
         maxXForSpline.set(x[1]);
      }
   }

   /**
    * initializes a spline based on the current information
    * @param supportLeg
    * @param spline
    * @return the minimum and maximum abscissa for which the spline is valid
    */
   private double[] initializeSpline(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, YoPolynomial spline, double offsetX,
         double offsetZ)
   {
      double x = getCurrentCoMX();

      double x0 = x - offsetX;
      double z0 = getDesiredCenterOfMassHeight() - offsetZ;
      double dzdx0 = getDesiredCenterOfMassHeightSlope().length();
      double d2zdx20 = 0.0;

      //TODO: Recheck that this is the right thing to do. 
      Footstep footstep = transferToAndNextFootstepsData.getTransferToFootstep();
      double xf = computeXf(supportLeg, footstep) - offsetX;
      double zf = findMinZOfGroundContactPoints(supportLeg) + nominalHeightAboveGround.getDoubleValue() - offsetZ;
      double dzdxf = 0.0;
      double d2zdx2f = 0.0;

      if (spline == heightSplineInFootFrame)
         deltaZ.set(zf - z0);

      spline.setQuintic(x0, xf, z0, dzdx0, d2zdx20, zf, dzdxf, d2zdx2f);

      return new double[] { x0, xf };
   }

   private void compute()
   {
      double x = getCurrentCoMX() - footX.getDoubleValue();
      heightSplineInFootFrame.compute(MathTools.clipToMinMax(x, minXForSpline.getDoubleValue(), maxXForSpline.getDoubleValue()));

      desiredComHeightInWorld.set(heightSplineInFootFrame.getPosition() + footZ.getDoubleValue());

      if (MathTools.isInsideBoundsInclusive(x, minXForSpline.getDoubleValue(), maxXForSpline.getDoubleValue()))
      {
         desiredComHeightSlope.set(heightSplineInFootFrame.getVelocity());
         desiredComHeightSecondDerivative.set(heightSplineInFootFrame.getAcceleration());

         centerOfMassJacobian.compute();
         FrameVector comVelocity = new FrameVector(referenceFrame);
         centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
         double xd = comVelocity.getX();
         orbitalEnergy.set(computeOrbitalEnergy(heightSplineInFootFrame, x, xd, gravityZ));
      }
      else
      {
         desiredComHeightSlope.set(0.0);
         desiredComHeightSecondDerivative.set(0.0);
         orbitalEnergy.set(Double.NaN);
      }
   }

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData)
   {
      compute();

      coMHeightPartialDerivativesDataToPack.setCoMHeight(ReferenceFrame.getWorldFrame(), desiredComHeightInWorld.getDoubleValue());
      coMHeightPartialDerivativesDataToPack.setPartialDzDx(desiredComHeightSlope.getDoubleValue());
      coMHeightPartialDerivativesDataToPack.setPartialDzDy(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDx2(desiredComHeightSecondDerivative.getDoubleValue());
      coMHeightPartialDerivativesDataToPack.setPartialD2zDy2(0.0);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDxDy(0.0);
   }

   private double getDesiredCenterOfMassHeight()
   {
      return desiredComHeightInWorld.getDoubleValue();
   }

   private FrameVector2d getDesiredCenterOfMassHeightSlope()
   {
      return new FrameVector2d(referenceFrame, desiredComHeightSlope.getDoubleValue(), 0.0);
   }

   public double computeOrbitalEnergyIfInitializedNow(RobotSide upcomingSupportLeg)
   {
      throwExceptionAndBail();

      double footX = findMaxXOfGroundContactPoints(upcomingSupportLeg);
      double footZ = findMinZOfGroundContactPoints(upcomingSupportLeg);

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = null;
      initializeSpline(transferToAndNextFootstepsData, upcomingSupportLeg, testHeightSplineInFootFrame, footX, footZ);

      double x = getCurrentCoMX() - footX;

      centerOfMassJacobian.compute();
      FrameVector comVelocity = new FrameVector(referenceFrame);
      centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(referenceFrame);
      double xd = comVelocity.getX();

      return computeOrbitalEnergy(testHeightSplineInFootFrame, x, xd, gravityZ);
   }

   private void throwExceptionAndBail()
   {
      throw new RuntimeException("This class needs lots of love. Talk to Jerry and Twan to get Steep stair climbing working again!");
   }

   private double computeOrbitalEnergy(YoPolynomial spline, double x, double xd, double g)
   {
      return OrbitalEnergyCalculator.computeOrbitalEnergy(spline, g, x, xd);
   }

   private double getCurrentCoMX()
   {
      FramePoint com = new FramePoint(centerOfMassFrame);
      com.changeFrame(referenceFrame);
      double x = com.getX();

      return x;
   }

   private double computeXf(RobotSide supportLeg, Footstep footstep)
   {
      double supportFootMaxX = findMaxXOfGroundContactPoints(supportLeg);
      double swingFootMaxX = findMaxXOfGroundContactPoints(footstep, supportLeg.getOppositeSide());

      return (supportFootMaxX + swingFootMaxX) / 2.0;
   }

   private double findMinZOfGroundContactPoints(RobotSide robotSide)
   {
      ContactablePlaneBody contactableBody = bipedFeet.get(robotSide);
      RigidBodyTransform footToWorldTransform = contactableBody.getFrameAfterParentJoint().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      FramePoint minZPoint = DesiredFootstepCalculatorTools.computeMinZPointInFrame(footToWorldTransform, contactableBody, referenceFrame);
      return minZPoint.getZ();
   }

   private double findMaxXOfGroundContactPoints(RobotSide robotSide)
   {
      ContactablePlaneBody contactableBody = bipedFeet.get(robotSide);
      RigidBodyTransform footToWorldTransform = contactableBody.getFrameAfterParentJoint().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      FramePoint maxXPoint = DesiredFootstepCalculatorTools.computeMaxXPointInFrame(footToWorldTransform, contactableBody, referenceFrame);
      return maxXPoint.getX();
   }

   private double findMaxXOfGroundContactPoints(Footstep footstep, RobotSide swingSide)
   {
      FramePose footstepPose = new FramePose();
      footstep.getPose(footstepPose);
      RigidBodyTransform desiredFootToDesiredHeading = new RigidBodyTransform();
      footstepPose.getPose(desiredFootToDesiredHeading);

      List<FramePoint> footPoints = bipedFeet.get(swingSide).getContactPointsCopy();
      double maxX = Double.NEGATIVE_INFINITY;
      for (FramePoint footPoint : footPoints)
      {
         footPoint.changeFrame(referenceFrames.getFootFrame(swingSide));

         footPoint.changeFrameUsingTransform(referenceFrame, desiredFootToDesiredHeading);
         if (footPoint.getX() > maxX)
            maxX = footPoint.getX();
      }

      return maxX;
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      return false;
   }

   public void attachWalkOnToesManager(WalkOnTheEdgesManager walkOnTheEdgesManager)
   {
   }

   @Override
   public void setSupportLeg(RobotSide supportLeg)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      // TODO Auto-generated method stub
      
   }
}