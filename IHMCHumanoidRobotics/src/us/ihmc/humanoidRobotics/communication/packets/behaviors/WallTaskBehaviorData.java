package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;

public class WallTaskBehaviorData extends Packet<WallTaskBehaviorData>
{
	public enum Commands{START, INSERT, CUT, RETRACT, READY_TO_DROP};
	public Commands command = null;
	
	public WallTaskBehaviorData()
	{
		// empty constructor
	}
	
	public WallTaskBehaviorData(Random random) {
		
		command = RandomNumbers.nextEnum(random, Commands.class);
	}
	
	public void start()
	{
		this.command = Commands.START;
	}
	
	public void cut()
	{
		this.command = Commands.CUT;
	}
	
	public void insert()
	{
		this.command = Commands.INSERT;
	}
	
	public void retract()
	{
		this.command = Commands.RETRACT;
	}
	
	public void drop()
	{
		this.command = Commands.READY_TO_DROP;
	}
	
	public Commands getCommand()
	{
		return command;
	}
	
	@Override
	public boolean epsilonEquals(WallTaskBehaviorData other, double epsilon) {
		return other.command == this.command;
	}

}
