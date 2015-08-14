package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class CutForceControlHelper {
	public static final double GRAVITY = 9.81;
	
	//TODO: Reference to confluence page of Master's Thesis
	private CutForceControlHelper()
	{
		
	}
	
	/**
	 * Reads the forces from the sensors, compensates for gravity, stores the values in yoVariables and updates filtered variables.
	 * @param wristSensorWrench
	 * @param worldFrame
	 * @param fxRaw
	 * @param fyRaw
	 * @param fzRaw
	 * @param fxFiltered
	 * @param fyFiltered
	 * @param fzFiltered
	 * @param massHandAndDrill
	 * @param adNoise
	 * @param randomNumberGenerator
	 */
	static void wristSensorUpdate(Wrench wristSensorWrench, ReferenceFrame worldFrame,
			DoubleYoVariable fxRaw, DoubleYoVariable fyRaw, DoubleYoVariable fzRaw,
			AlphaFilteredYoVariable fxFiltered, AlphaFilteredYoVariable fyFiltered, AlphaFilteredYoVariable fzFiltered,
			double massHandAndDrill, boolean adNoise, Random randomNumberGenerator)
	{
		if (wristSensorWrench.getExpressedInFrame() != worldFrame)
		{
			wristSensorWrench.changeFrame(worldFrame);
		}

		if (adNoise)
		{
			addWhiteNoise(wristSensorWrench, 0.5, randomNumberGenerator);
		}

		fxRaw.set(wristSensorWrench.getLinearPartX());
		fyRaw.set(wristSensorWrench.getLinearPartY());
		fzRaw.set(wristSensorWrench.getLinearPartZ());
		fzRaw.add(massHandAndDrill * GRAVITY);
		
		/**
		 * For higher velocities the radial force must be compensated as well!!
		 */
//		if (compensateRadialForce.getBooleanValue() && circularTrajectoryOrigin != null
//				&& timeParameterScaleFactor.getDoubleValue() > 0.0)
//		{
//			//angular velocity
//			circularTrajectoryRadius.sub(currentPositionInWorld, circularTrajectoryOrigin);
//			circularTrajectoryRadius.setX(0.0);
//			
//			if(currentTangVelocity > 0.0)
//			{
//				circularTrajectoryOmega.normalize();
//				circularTrajectoryOmega.scale(currentTangVelocity);
//				// radial acc az = omega cross (omega cross r)
//				tempVector2.cross(circularTrajectoryOmega, circularTrajectoryRadius);
//				tempVector.cross(circularTrajectoryOmega, tempVector2);
//				
//				tempVector.scale(massHandandDrill.getDoubleValue());
//				fxRaw.add(tempVector.getX());
//				fyRaw.add(tempVector.getY());
//				fzRaw.add(tempVector.getZ());
//			}
//		}

		fxFiltered.update();
		fyFiltered.update();
		fzFiltered.update();
	}
	
	/**
	 * Projects the filtered forces in world coordinates onto the tangent vector.
	 * @param forceVectorInWorld
	 * @param currentTangentialForce
	 * @param tangentTrajectoryVectorInWorld
	 * @param lastTangentTrajectoryVectorInWorld
	 * @param fxFiltered
	 * @param fyFiltered
	 * @param fzFiltered
	 * @param addDisturbances
	 * @param timeForDisturbances
	 */
	static void getTangentForce(Vector3d forceVectorInWorld, DoubleYoVariable currentTangentialForce,
			Vector3d tangentTrajectoryVectorInWorld, Vector3d lastTangentTrajectoryVectorInWorld,
			Double fxFiltered, Double fyFiltered, Double fzFiltered,
			boolean addDisturbances, double timeForDisturbances)
	{
		forceVectorInWorld.set(fxFiltered, fyFiltered, fzFiltered);

		if (tangentTrajectoryVectorInWorld.length() > 0.0)
		{
			tangentTrajectoryVectorInWorld.normalize();
			lastTangentTrajectoryVectorInWorld.set(tangentTrajectoryVectorInWorld);
			currentTangentialForce.set(forceVectorInWorld.dot(tangentTrajectoryVectorInWorld));
			if (addDisturbances)
			{
				currentTangentialForce.add(generateSinusoidalDisturbance(10.0, 0.1, timeForDisturbances, Math.PI / 3.0));
				currentTangentialForce.add(generateDriftDisturbance(0.01, timeForDisturbances));
			}
		}
		else
		{
			currentTangentialForce.set(0.0);
		}
	}

	/**
	 * Returns the cutting force as a function of the velocity with the parameters C1 and C2.
	 * @param cutVelocity
	 * @param C1
	 * @param C2
	 * @return
	 */
	static double exponentialForceModel(double cutVelocity, double C1, double C2)
	{
		return C1 * ( Math.exp(C2 * cutVelocity) - 1.0);
	}

	/**
	 * Ramp function that increases linearly from y(x=xStart) = 0 to y(x=xEnd) = 1.
	 * The parameters w**nom will be weighted with the value of the ramp and stored in the corresponding YoVariables w**
	 * So far only implemented for 0 > xstart >xend
	 * @param x
	 * @param xstart
	 * @param xend
	 * @param w11Nom
	 * @param w22Nom
	 * @param w11
	 * @param w22
	 */
	static void adaptW(double x, double xStart, double xEnd, double w11Nom, double w22Nom,
			DoubleYoVariable w11, DoubleYoVariable w22)
	{
		/**
		 * Sets the weighting coefficients of the matrix W.
		 * Ramp starting from xStart to xEnd, increasing from 0 to 1.
		 */
		if (x < xStart)
		{
			w11.set(0.0);
			w22.set(0.0);
		}
		else if (x > xEnd)
		{
			w11.set(w11Nom);
			w22.set(w22Nom);
		}
		else
		{
			w11.set(w11Nom  / (xEnd - xStart) * (x - xStart) );
			w22.set(w22Nom / (xEnd - xStart) * (x - xStart) );
		}
	}
	
	/**
	 * Updates the current model parameters using the gradient search method introduced in MasterThesisReport
	 * @param currentTangentialForce
	 * @param currentTangentialForceModel
	 * @param currentTangentialVelocity
	 * @param epsilon
	 * @param C1
	 * @param C2
	 * @param w1
	 * @param w2
	 */
	static void modelParameterAdaption(double currentTangentialForce, double currentTangentialForceModel,
			double currentTangentialVelocity, DoubleYoVariable epsilon, DoubleYoVariable C1, DoubleYoVariable C2,
			double w1, double w2)
	{
		epsilon.set(Math.log(Math.abs(currentTangentialForce) + C1.getDoubleValue())
				- Math.log(currentTangentialForceModel + C1.getDoubleValue()));
		
		double lnC1 = Math.log(C1.getDoubleValue());
		double c2 = C2.getDoubleValue();
		
		lnC1 += w1 * epsilon.getDoubleValue();
		c2 += w2 * c2 * epsilon.getDoubleValue() * currentTangentialVelocity;
		C1.set(Math.exp(lnC1));
		C2.set(c2);
		
		if(C2.getDoubleValue() < 0.001)
		{
			C2.set(0.001);
		}
		
		if(C1.getDoubleValue() < 0.001)
		{
			C1.set(0.001);
		}
	}
	
	/**
	 * Function that adds white noise to the linear elements of a wrench.
	 * @param wrench
	 * @param linearAmp
	 * @param randomNumberGenerator
	 */
	static void addWhiteNoise(Wrench wrench, double linearAmp, Random randomNumberGenerator)
	{
		double x = wrench.getLinearPartX() + randomNumberGenerator.nextDouble() * linearAmp;
		double y = wrench.getLinearPartY() + randomNumberGenerator.nextDouble() * linearAmp;
		double z = wrench.getLinearPartZ() + randomNumberGenerator.nextDouble() * linearAmp;
		
		wrench.setLinearPartX(x);
		wrench.setLinearPartY(y);
		wrench.setLinearPartZ(z);
	}
	
	/**
	 * Function that returns the value of a sinusoidal.
	 * @param amp
	 * @param freq
	 * @param time
	 * @param phase
	 * @return
	 */
	static double generateSinusoidalDisturbance(double amp, double freq, double time, double phase)
	{
		return Math.sin(2.0 * Math.PI * freq * time + phase % (Math.PI * 2.0));
	}
	
	/**
	 * Function that returns the value of a constant slope starting at time=0.
	 * @param slope
	 * @param time
	 * @return
	 */
	static double generateDriftDisturbance(double slope, double time)
	{
		return slope * time;
	}

	/**
	 * Returns a step function of height amp when time>=startime.
	 * @param amp
	 * @param time
	 * @param starttime
	 * @return
	 */
	static double step(double amp, double time, double starttime)
	{
		if (time < starttime)
		{
			return 0.0;
		}
		else
		{
			return -amp;
		}
	}
	
}