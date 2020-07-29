package us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI;

import controller_msgs.msg.dds.DirectionalControlInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class DirectionalControlInputCommand
		implements Command<DirectionalControlInputCommand, DirectionalControlInputMessage> {
	private long sequenceId;
	private DirectionalControlInputMessage message;

	@Override
	public void set(DirectionalControlInputCommand other) {
		clear();

		sequenceId = other.sequenceId;
	}

	@Override
	public void clear() {
		sequenceId = 0;
	}

	@Override
	public void setFromMessage(DirectionalControlInputMessage message) {
		set(message);
	}

	public void set(DirectionalControlInputMessage message) {
		clear();
		sequenceId = message.getSequenceId();
		this.message = message;
	}

	@Override
	public Class<DirectionalControlInputMessage> getMessageClass() {
		return DirectionalControlInputMessage.class;
	}

	@Override
	public boolean isCommandValid() {
		return true;
	}

	@Override
	public long getSequenceId() {
		return sequenceId;
	}

	public DirectionalControlInputMessage getMessage() {
		return message;
	}
}
