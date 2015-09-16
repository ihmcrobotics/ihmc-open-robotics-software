package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import org.apache.commons.lang3.RandomStringUtils;

import us.ihmc.communication.packets.Packet;

public class LocalizationStatusPacket extends Packet<LocalizationStatusPacket>
{
	public double overlap;
	public String status;
	
	public LocalizationStatusPacket()
	{
		// for serialization
	}
	
	public LocalizationStatusPacket(double overlap, String status)
	{
		this.overlap = overlap;
		this.status = status;
	}
	
	public double getOverlap()
	{
		return overlap;
	}
	
	public String getStatus()
	{
		return status;
	}

	@Override
	public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon) {
	    return (Math.abs(other.getOverlap() - this.getOverlap()) < epsilon) && (other.getStatus().equals(this.getStatus()));
	}
	
	public LocalizationStatusPacket(Random random)
	{
		int length = random.nextInt(255);
		
		this.overlap = random.nextDouble();
		this.status = RandomStringUtils.random(length, true, true);
	}
}
