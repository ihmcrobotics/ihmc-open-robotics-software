package us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI;

import controller_msgs.msg.dds.DirectionalControlConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.log.LogTools;

public class DirectionalControlConfigurationCommand
		implements Command<DirectionalControlConfigurationCommand, DirectionalControlConfigurationMessage> {
	private long sequenceId;
	private DirectionalControlConfigurationMessage message;

	@Override
	public void set(DirectionalControlConfigurationCommand other) {
		clear();

		sequenceId = other.sequenceId;
	}

	@Override
	public void clear() {
		sequenceId = 0;
	}

	@Override
	public void setFromMessage(DirectionalControlConfigurationMessage message) {
		LogTools.info("setFromMessage has: " + message.toString());
		set(message);
	}

	public void set(DirectionalControlConfigurationMessage message) {
		clear();
		sequenceId = message.getSequenceId();
		this.message = message;
	}

	@Override
	public Class<DirectionalControlConfigurationMessage> getMessageClass() {
		return DirectionalControlConfigurationMessage.class;
	}

	@Override
	public boolean isCommandValid() {
		return true;
	}

	@Override
	public long getSequenceId() {
		return sequenceId;
	}

	public DirectionalControlConfigurationMessage getMessage() {
		return message;
	}
}
