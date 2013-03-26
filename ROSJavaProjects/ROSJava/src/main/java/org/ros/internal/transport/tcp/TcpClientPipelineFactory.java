/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.internal.transport.tcp;

import org.jboss.netty.channel.ChannelPipeline;
import org.jboss.netty.channel.group.ChannelGroup;
import org.jboss.netty.handler.codec.frame.LengthFieldBasedFrameDecoder;
import org.jboss.netty.handler.codec.frame.LengthFieldPrepender;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TcpClientPipelineFactory extends ConnectionTrackingChannelPipelineFactory {

  public static final String LENGTH_FIELD_BASED_FRAME_DECODER = "LengthFieldBasedFrameDecoder";
  public static final String LENGTH_FIELD_PREPENDER = "LengthFieldPrepender";

  public TcpClientPipelineFactory(ChannelGroup channelGroup) {
    super(channelGroup);
  }

  @Override
  public ChannelPipeline getPipeline() {
    ChannelPipeline pipeline = super.getPipeline();
    pipeline.addLast(LENGTH_FIELD_PREPENDER, new LengthFieldPrepender(4));
    pipeline.addLast(LENGTH_FIELD_BASED_FRAME_DECODER, new LengthFieldBasedFrameDecoder(
        Integer.MAX_VALUE, 0, 4, 0, 4));
    return pipeline;
  }
}
